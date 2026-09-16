// RealSense D436 适配器：对齐深度、复制帧数据并转换为米制。
#include "camera_driver/realsense_camera_driver.hpp"

#include <algorithm>
#include <cstring>
#include <exception>

namespace camera_driver {
using namespace camera_adapter;

void RealSenseCameraDriver::initialize(const CameraConfig &config) {
    config.validate();
    if (config.backend != "realsense") {
        throw CameraError(ErrorCode::InvalidConfig, "RealSense 驱动收到的 backend 不是 realsense");
    }
#ifndef CAMERA_DRIVER_HAS_REALSENSE
    (void)config;
    throw CameraError(ErrorCode::UnsupportedBackend,
                      "当前构建未启用 RealSense SDK，请安装 librealsense 后打开 "
                      "CAMERA_DRIVER_ENABLE_REALSENSE");
#else
    try {
        std::lock_guard<std::mutex> lock(mutex_);
        config_ = config;
        pipeline_config_ = rs2::config();
        if (!config.serial.empty()) {
            pipeline_config_.enable_device(config.serial);
        }
        pipeline_config_.enable_stream(RS2_STREAM_COLOR, static_cast<int>(config.color_width),
                                       static_cast<int>(config.color_height), RS2_FORMAT_RGB8,
                                       static_cast<int>(config.fps));
        pipeline_config_.enable_stream(RS2_STREAM_DEPTH, static_cast<int>(config.depth_width),
                                       static_cast<int>(config.depth_height), RS2_FORMAT_Z16,
                                       static_cast<int>(config.fps));
        instance_ = new_instance_id();
        sequence_ = 0;
        initialized_ = true;
        running_ = false;
    } catch (const std::exception &error) {
        throw CameraError(ErrorCode::CameraNotFound,
                          std::string("RealSense 初始化失败: ") + error.what());
    }
#endif
}

void RealSenseCameraDriver::start() {
#ifndef CAMERA_DRIVER_HAS_REALSENSE
    throw CameraError(ErrorCode::UnsupportedBackend, "RealSense SDK 未启用");
#else
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
        throw CameraError(ErrorCode::NotReady, "RealSense 驱动尚未初始化");
    }
    try {
        pipeline_profile_ = pipeline_.start(pipeline_config_);
        align_to_color_ = std::make_unique<rs2::align>(RS2_STREAM_COLOR);
        const auto device = pipeline_profile_.get_device();
        description_.serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        description_.model = device.get_info(RS2_CAMERA_INFO_NAME);
        // D436 配置只打开 RGB-D；若部署的是带 IMU 的型号，应另加显式能力探测。
        description_.imu_supported = false;
        description_.active_fps = config_.fps;
        instance_ = new_instance_id();
        sequence_ = 0;
        running_ = true;
    } catch (const std::exception &error) {
        throw CameraError(ErrorCode::DeviceFailure,
                          std::string("RealSense 启动失败: ") + error.what());
    }
#endif
}

#ifdef CAMERA_DRIVER_HAS_REALSENSE
std::shared_ptr<CameraFrame> RealSenseCameraDriver::make_frame(
    const rs2::frameset &input, uint64_t receive_steady_ns,
    uint64_t receive_system_ns) {
    if (!align_to_color_) {
        throw CameraError(ErrorCode::NotReady, "RealSense 对齐器尚未初始化");
    }
    const rs2::frameset frameset = align_to_color_->process(input);
    const rs2::video_frame color = frameset.get_color_frame();
    const rs2::depth_frame depth = frameset.get_depth_frame();
    if (!color || !depth) {
        throw CameraError(ErrorCode::FrameNotFound, "RealSense 帧组缺少 RGB 或深度帧");
    }
    const int width = color.get_width();
    const int height = color.get_height();
    if (width <= 0 || height <= 0 || depth.get_width() != width || depth.get_height() != height) {
        throw CameraError(ErrorCode::InvalidCalibration, "RealSense 对齐后的 RGB-D 尺寸不一致");
    }
    const size_t pixels = static_cast<size_t>(width) * static_cast<size_t>(height);
    auto result = std::make_shared<CameraFrame>();
    result->provider_instance = instance_;
    result->sequence = ++sequence_;
    result->camera_name = config_.name;
    result->optical_frame = config_.optical_frame;
    result->calibration_revision = config_.calibration_revision;
    result->intrinsics.width = static_cast<uint32_t>(width);
    result->intrinsics.height = static_cast<uint32_t>(height);
    const auto color_profile = color.get_profile().as<rs2::video_stream_profile>();
    const auto intrinsics = color_profile.get_intrinsics();
    result->intrinsics.fx = intrinsics.fx;
    result->intrinsics.fy = intrinsics.fy;
    result->intrinsics.cx = intrinsics.ppx;
    result->intrinsics.cy = intrinsics.ppy;
    switch (intrinsics.model) {
        case RS2_DISTORTION_INVERSE_BROWN_CONRADY:
            result->intrinsics.distortion_model = "inverse_brown_conrady";
            break;
        case RS2_DISTORTION_MODIFIED_BROWN_CONRADY:
            result->intrinsics.distortion_model = "modified_brown_conrady";
            break;
        default:
            result->intrinsics.distortion_model = "plumb_bob";
            break;
    }
    for (size_t index = 0; index < 5; ++index) {
        result->intrinsics.distortion[index] = intrinsics.coeffs[index];
    }
    result->rgb.resize(pixels * 3);
    const auto *rgb_source = static_cast<const uint8_t *>(color.get_data());
    const int rgb_stride = color.get_stride_in_bytes();
    for (int row = 0; row < height; ++row) {
        std::memcpy(result->rgb.data() + static_cast<size_t>(row) * width * 3,
                    rgb_source + static_cast<size_t>(row) * rgb_stride,
                    static_cast<size_t>(width) * 3);
    }
    result->depth_m.resize(pixels);
    const auto *depth_source = static_cast<const uint16_t *>(depth.get_data());
    const int depth_stride = depth.get_stride_in_bytes();
    const float depth_scale = depth.get_units();
    for (int row = 0; row < height; ++row) {
        const auto *source = reinterpret_cast<const uint16_t *>(
            reinterpret_cast<const uint8_t *>(depth_source) + static_cast<size_t>(row) * depth_stride);
        for (int column = 0; column < width; ++column) {
            result->depth_m[static_cast<size_t>(row) * width + column] =
                static_cast<float>(source[column]) * depth_scale;
        }
    }
    const double color_ms = color.get_timestamp();
    const double depth_ms = depth.get_timestamp();
    const bool comparable = color.get_frame_timestamp_domain() == depth.get_frame_timestamp_domain();
    result->timing = receive_timing(
        config_, receive_steady_ns, receive_system_ns,
        color_ms > 0.0 ? static_cast<uint64_t>(color_ms * 1000.0) : 0,
        depth_ms > 0.0 ? static_cast<uint64_t>(depth_ms * 1000.0) : 0,
        comparable);
    result->depth_aligned_to_color = true;
    return result;
}
#endif

std::shared_ptr<CameraFrame> RealSenseCameraDriver::wait_frame(uint32_t timeout_ms) {
#ifndef CAMERA_DRIVER_HAS_REALSENSE
    (void)timeout_ms;
    throw CameraError(ErrorCode::UnsupportedBackend, "RealSense SDK 未启用");
#else
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
        throw CameraError(ErrorCode::NotReady, "RealSense 驱动未启动");
    }
    try {
        const auto frameset = pipeline_.wait_for_frames(static_cast<unsigned int>(timeout_ms));
        // 接收时间必须在 SDK 返回帧之后采样，避免把阻塞等待时间计入帧年龄。
        const uint64_t receive_steady = steady_now_ns();
        const uint64_t receive_system = system_now_ns();
        return make_frame(frameset, receive_steady, receive_system);
    } catch (const CameraError &) {
        throw;
    } catch (const std::exception &error) {
        throw CameraError(ErrorCode::DeviceFailure,
                          std::string("RealSense 取帧失败: ") + error.what());
    }
#endif
}

DeviceDescription RealSenseCameraDriver::description() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return description_;
}

void RealSenseCameraDriver::stop() noexcept {
#ifdef CAMERA_DRIVER_HAS_REALSENSE
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) {
        try {
            pipeline_.stop();
        } catch (...) {
        }
    }
    align_to_color_.reset();
#endif
    running_ = false;
}

}  // namespace camera_driver
