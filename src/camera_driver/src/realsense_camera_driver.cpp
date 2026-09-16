// RealSense D436 适配器：对齐深度、复制帧数据并转换为米制。
#include "camera_driver/realsense_camera_driver.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>

namespace camera_driver {
using namespace camera_adapter;

// 校验配置并准备RealSense stream config；initialize不创建streaming session UUID。
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
        instance_ = {};  // v0.2.4：只有start成功才建立新的streaming session。
        sequence_ = 0;
        pipeline_config_ = rs2::config();
        // 多设备部署必须按真实序列号绑定，禁止枚举顺序漂移。
        if (!config.serial.empty()) {
            pipeline_config_.enable_device(config.serial);
        }
        // 统一Color输出RGB8。
        pipeline_config_.enable_stream(RS2_STREAM_COLOR, static_cast<int>(config.color_width),
                                       static_cast<int>(config.color_height), RS2_FORMAT_RGB8,
                                       static_cast<int>(config.fps));
        // RealSense原始深度使用Z16，随后由get_units转换成米制float32。
        pipeline_config_.enable_stream(RS2_STREAM_DEPTH, static_cast<int>(config.depth_width),
                                       static_cast<int>(config.depth_height), RS2_FORMAT_Z16,
                                       static_cast<int>(config.fps));
        initialized_ = true;
        running_ = false;
    } catch (const std::exception &error) {
        throw CameraError(ErrorCode::CameraNotFound,
                          std::string("RealSense 初始化失败: ") + error.what());
    }
#endif
}

// 启动Pipeline、创建D2C aligner并读取真实设备身份；成功后生成新session UUID。
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
        // v0.2.4统一语义：每次start成功生成一次新session UUID，sequence从0重新开始。
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
// 将RealSense frameset做Depth->Color对齐并复制成厂商无关CameraFrame。
std::shared_ptr<CameraFrame> RealSenseCameraDriver::make_frame(
    const rs2::frameset &input, uint64_t receive_steady_ns,
    uint64_t receive_system_ns) {
    if (!align_to_color_) {
        throw CameraError(ErrorCode::NotReady, "RealSense 对齐器尚未初始化");
    }
    // 软件把Depth重采样到Color像素网格。
    const rs2::frameset frameset = align_to_color_->process(input);
    const rs2::video_frame color = frameset.get_color_frame();
    const rs2::depth_frame depth = frameset.get_depth_frame();
    if (!color || !depth) {
        throw CameraError(ErrorCode::FrameNotFound, "RealSense 帧组缺少 RGB 或深度帧");
    }
    // 对齐后的Color/Depth必须同尺寸。
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

    // 使用Color profile内参，因为Depth已经对齐到Color坐标系。
    result->intrinsics.width = static_cast<uint32_t>(width);
    result->intrinsics.height = static_cast<uint32_t>(height);
    const auto color_profile = color.get_profile().as<rs2::video_stream_profile>();
    const auto intrinsics = color_profile.get_intrinsics();
    result->intrinsics.fx = intrinsics.fx;
    result->intrinsics.fy = intrinsics.fy;
    result->intrinsics.cx = intrinsics.ppx;
    result->intrinsics.cy = intrinsics.ppy;
    // 将RealSense畸变枚举映射到统一字符串模型。
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
    // librealsense当前返回5个主要系数，统一数组剩余项保持0。
    for (size_t index = 0; index < 5; ++index) {
        result->intrinsics.distortion[index] = intrinsics.coeffs[index];
    }

    // 按行复制RGB，兼容SDK stride大于width*3的情况。
    result->rgb.resize(pixels * 3);
    const auto *rgb_source = static_cast<const uint8_t *>(color.get_data());
    const int rgb_stride = color.get_stride_in_bytes();
    for (int row = 0; row < height; ++row) {
        std::memcpy(result->rgb.data() + static_cast<size_t>(row) * width * 3,
                    rgb_source + static_cast<size_t>(row) * rgb_stride,
                    static_cast<size_t>(width) * 3);
    }

    // 深度按get_units转换为米，并在入帧阶段执行统一有效范围过滤。
    result->depth_m.resize(pixels);
    const auto *depth_source = static_cast<const uint16_t *>(depth.get_data());
    const int depth_stride = depth.get_stride_in_bytes();
    const float depth_scale = depth.get_units();
    for (int row = 0; row < height; ++row) {
        const auto *source = reinterpret_cast<const uint16_t *>(
            reinterpret_cast<const uint8_t *>(depth_source) + static_cast<size_t>(row) * depth_stride);
        for (int column = 0; column < width; ++column) {
            const float depth_m = static_cast<float>(source[column]) * depth_scale;
            const size_t index = static_cast<size_t>(row) * width + column;
            // v0.2.4统一约定：范围外、非有限或0深度全部存为0。
            result->depth_m[index] =
                std::isfinite(depth_m) && depth_m >= config_.min_depth_m && depth_m <= config_.max_depth_m
                    ? depth_m
                    : 0.0F;
        }
    }

    // RealSense timestamp单位为毫秒，转换成微秒进入统一FrameTiming。
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

// 等待SDK帧组并在返回后立即采样本机接收时间。
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

// 返回真实设备描述快照。
DeviceDescription RealSenseCameraDriver::description() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return description_;
}

// 尽力停止RealSense Pipeline；退出路径保持noexcept。
void RealSenseCameraDriver::stop() noexcept {
#ifdef CAMERA_DRIVER_HAS_REALSENSE
    try {
        std::lock_guard<std::mutex> lock(mutex_);
        if (running_) {
            try {
                pipeline_.stop();
            } catch (...) {
                // SDK停止异常不能破坏ROS2析构路径。
            }
        }
        align_to_color_.reset();
        running_ = false;
        instance_ = {};
    } catch (...) {
        running_ = false;
    }
#else
    running_ = false;
#endif
}

}  // namespace camera_driver
