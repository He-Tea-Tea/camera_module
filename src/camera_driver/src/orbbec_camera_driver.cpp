// Orbbec Gemini335Le 适配器；厂商对象只在本文件内出现。
#include "camera_driver/orbbec_camera_driver.hpp"

#include <algorithm>
#include <cstring>
#include <exception>
#include <iostream>

namespace camera_driver {
using namespace camera_adapter;

void OrbbecCameraDriver::initialize(const CameraConfig &config) {
    config.validate();
    if (config.backend != "orbbec") {
        throw CameraError(ErrorCode::InvalidConfig, "Orbbec 驱动收到的 backend 不是 orbbec");
    }
#ifndef CAMERA_DRIVER_HAS_ORBBEC
    (void)config;
    throw CameraError(ErrorCode::UnsupportedBackend,
                      "当前构建未启用 Orbbec SDK，请安装 SDK 后打开 CAMERA_DRIVER_ENABLE_ORBBEC");
#else
    try {
        std::lock_guard<std::mutex> lock(mutex_);
        config_ = config;
        std::clog << "[orbbec_driver] [" << config.name
                  << "] 创建 Orbbec SDK Context" << std::endl;
        context_ = std::make_unique<ob::Context>();
        if (!config.ip_address.empty()) {
            std::clog << "[orbbec_driver] [" << config.name
                      << "] 连接网络设备 " << config.ip_address
                      << ":" << config.network_port << std::endl;
            // 固定IP连接直接调用createNetDevice，不启动全网段设备枚举线程。
            // SDK内部会主动查询指定地址，避免网络枚举器影响启动和退出。
            device_ = context_->createNetDevice(config.ip_address.c_str(), config.network_port);
        } else {
            std::clog << "[orbbec_driver] [" << config.name
                      << "] 枚举本机 Orbbec 设备" << std::endl;
            auto devices = context_->queryDeviceList();
            if (devices->getCount() == 0) {
                throw CameraError(ErrorCode::CameraNotFound, "没有发现 Orbbec 设备");
            }
            device_ = config.serial.empty()
                          ? devices->getDevice(0)
                          : devices->getDeviceBySN(config.serial.c_str());
        }
        if (!device_) {
            throw CameraError(ErrorCode::CameraNotFound, "无法打开指定的 Orbbec 设备");
        }
        if (device_->isGlobalTimestampSupported()) {
            device_->enableGlobalTimestamp(true);
        }
        const auto info = device_->getDeviceInfo();
        const std::string actual_serial = info->getSerialNumber();
        if (!config.serial.empty() && actual_serial != config.serial) {
            throw CameraError(
                ErrorCode::InvalidConfig,
                "Orbbec 实际序列号 " + actual_serial +
                    " 与配置序列号 " + config.serial + " 不一致");
        }
        std::clog << "[orbbec_driver] [" << config.name
                  << "] 已打开设备 model=" << info->getName()
                  << " serial=" << actual_serial << std::endl;

        pipeline_ = std::make_unique<ob::Pipeline>(device_);
        pipeline_config_ = std::make_shared<ob::Config>();

        // 配置彩色码流。
        // Gemini335Le 实测支持 640x400 @ 30 FPS / RGB。
        pipeline_config_->enableVideoStream(
            OB_STREAM_COLOR,
            config.color_width,
            config.color_height,
            config.fps,
            OB_FORMAT_RGB
        );

        // 配置深度码流。
        // Gemini335Le 实测支持 640x400 @ 30 FPS / Y16。
        pipeline_config_->enableVideoStream(
            OB_STREAM_DEPTH,
            config.depth_width,
            config.depth_height,
            config.fps,
            OB_FORMAT_Y16
        );

        // 要求 SDK 只有在 Color 和 Depth 都存在时才输出 FrameSet。
        // 否则 waitForFrameset() 可能返回只包含单一流的 FrameSet，
        // 上层会错误地把正常的帧聚合过程判断为设备故障。
        pipeline_config_->setFrameAggregateOutputMode(
            OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE
        );

        // 软件执行 Depth -> Color 对齐。
        // 对齐后的 Depth 应与 Color 使用相同图像尺寸。
        pipeline_config_->setAlignMode(
            ALIGN_D2C_SW_MODE
        );

        instance_ = new_instance_id();
        sequence_ = 0;
        instance_ = new_instance_id();
        sequence_ = 0;
        description_.serial = actual_serial;
        description_.model = info->getName();
        description_.imu_supported = false;
        description_.active_fps = config.fps;
        initialized_ = true;
        running_ = false;
        std::clog << "[orbbec_driver] [" << config.name
                  << "] RGB-D Profile 配置完成" << std::endl;
    } catch (const CameraError &) {
        throw;
    } catch (const std::exception &error) {
        throw CameraError(ErrorCode::CameraNotFound,
                          std::string("Orbbec 初始化失败: ") + error.what());
    }
#endif
}

void OrbbecCameraDriver::start() {
#ifndef CAMERA_DRIVER_HAS_ORBBEC
    throw CameraError(ErrorCode::UnsupportedBackend, "Orbbec SDK 未启用");
#else
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || !pipeline_ || !pipeline_config_) {
        throw CameraError(ErrorCode::NotReady, "Orbbec 驱动尚未初始化");
    }
    try {
        std::clog << "[orbbec_driver] [" << config_.name
                  << "] 启用帧同步并启动 Pipeline" << std::endl;
        pipeline_->enableFrameSync();
        pipeline_->start(pipeline_config_);
        instance_ = new_instance_id();
        sequence_ = 0;
        running_ = true;
        std::clog << "[orbbec_driver] [" << config_.name
                  << "] Pipeline 启动成功" << std::endl;
    } catch (const std::exception &error) {
        throw CameraError(ErrorCode::DeviceFailure,
                          std::string("Orbbec 启动失败: ") + error.what());
    }
#endif
}

#ifdef CAMERA_DRIVER_HAS_ORBBEC
std::shared_ptr<CameraFrame> OrbbecCameraDriver::make_frame(
    const std::shared_ptr<ob::FrameSet> &frameset, uint64_t receive_steady_ns,
    uint64_t receive_system_ns) {
    const auto color = frameset->getColorFrame();
    const auto depth = frameset->getDepthFrame();
    if (!color || !depth) {
        throw CameraError(ErrorCode::FrameNotFound, "Orbbec 帧组缺少 RGB 或深度帧");
    }
    const uint32_t width = color->getWidth();
    const uint32_t height = color->getHeight();
    if (width == 0 || height == 0 || depth->getWidth() != width || depth->getHeight() != height) {
        throw CameraError(ErrorCode::InvalidCalibration, "Orbbec 对齐后的 RGB-D 尺寸不一致");
    }
    const size_t pixels = static_cast<size_t>(width) * height;
    if (color->getDataSize() < pixels * 3 || depth->getDataSize() < pixels * 2) {
        throw CameraError(ErrorCode::FrameNotFound, "Orbbec 帧数据长度不足");
    }
    auto result = std::make_shared<CameraFrame>();
    result->provider_instance = instance_;
    result->sequence = ++sequence_;
    result->camera_name = config_.name;
    result->optical_frame = config_.optical_frame;
    result->calibration_revision = config_.calibration_revision;
    result->intrinsics.width = width;
    result->intrinsics.height = height;
    const auto profile = color->getStreamProfile()->as<ob::VideoStreamProfile>();
    const auto intrinsic = profile->getIntrinsic();
    const auto distortion = profile->getDistortion();
    result->intrinsics.fx = intrinsic.fx;
    result->intrinsics.fy = intrinsic.fy;
    result->intrinsics.cx = intrinsic.cx;
    result->intrinsics.cy = intrinsic.cy;
    result->intrinsics.distortion_model = "plumb_bob";
    result->intrinsics.distortion = {distortion.k1, distortion.k2, distortion.p1,
                                     distortion.p2, distortion.k3, distortion.k4,
                                     distortion.k5, distortion.k6};
    result->rgb.assign(color->getData(), color->getData() + pixels * 3);
    const auto *raw_depth = reinterpret_cast<const uint16_t *>(depth->getData());
    const float scale_m = depth->getValueScale() * 0.001F;
    result->depth_m.resize(pixels);
    for (size_t index = 0; index < pixels; ++index) {
        result->depth_m[index] = static_cast<float>(raw_depth[index]) * scale_m;
    }
    const uint64_t color_ts = color->getGlobalTimeStampUs() != 0
                                  ? color->getGlobalTimeStampUs()
                                  : color->getTimeStampUs();
    const uint64_t depth_ts = depth->getGlobalTimeStampUs() != 0
                                  ? depth->getGlobalTimeStampUs()
                                  : depth->getTimeStampUs();
    result->timing = receive_timing(config_, receive_steady_ns, receive_system_ns,
                                    color_ts, depth_ts, color_ts != 0 && depth_ts != 0);
    result->depth_aligned_to_color = true;
    return result;
}
#endif

std::shared_ptr<CameraFrame> OrbbecCameraDriver::wait_frame(uint32_t timeout_ms) {
#ifndef CAMERA_DRIVER_HAS_ORBBEC
    (void)timeout_ms;
    throw CameraError(ErrorCode::UnsupportedBackend, "Orbbec SDK 未启用");
#else
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || !pipeline_) {
        throw CameraError(ErrorCode::NotReady, "Orbbec 驱动未启动");
    }
    try {
        const auto frameset = pipeline_->waitForFrameset(timeout_ms);
        if (!frameset) {
            throw CameraError(ErrorCode::Timeout, "Orbbec 等待帧超时");
        }
        // 接收时间必须在 SDK 返回帧之后采样，不能把等待开始时间当作接收时间。
        const uint64_t receive_steady = steady_now_ns();
        const uint64_t receive_system = system_now_ns();
        return make_frame(frameset, receive_steady, receive_system);
    } catch (const CameraError &) {
        throw;
    } catch (const std::exception &error) {
        throw CameraError(ErrorCode::DeviceFailure,
                          std::string("Orbbec 取帧失败: ") + error.what());
    }
#endif
}

DeviceDescription OrbbecCameraDriver::description() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return description_;
}

void OrbbecCameraDriver::stop() noexcept {
#ifdef CAMERA_DRIVER_HAS_ORBBEC
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_ && pipeline_) {
        try {
            pipeline_->stop();
        } catch (...) {
        }
    }
#endif
    running_ = false;
}

}  // namespace camera_driver
