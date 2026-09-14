// 确定性 Mock 驱动：模拟采集节拍、无效深度和断连。
#include "camera_driver/mock_camera_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace camera_driver {
using namespace camera_adapter;

void MockCameraDriver::initialize(const CameraConfig &config) {
    config.validate();
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
    instance_ = new_instance_id();
    sequence_ = 0;
    next_frame_steady_ns_ = 0;
    description_.serial = config.serial.empty() ? "MOCK-" + config.name : config.serial;
    description_.model = config.model;
    description_.imu_supported = false;
    description_.active_fps = config.fps;
    initialized_.store(true);
    running_.store(false);
}

void MockCameraDriver::start() {
    if (!initialized_.load()) {
        throw CameraError(ErrorCode::NotReady, "Mock 驱动尚未初始化");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    instance_ = new_instance_id();
    sequence_ = 0;
    next_frame_steady_ns_ = steady_now_ns();
    running_.store(true);
}

std::shared_ptr<CameraFrame> MockCameraDriver::wait_frame(uint32_t timeout_ms) {
    if (!running_.load()) {
        throw CameraError(ErrorCode::NotReady, "Mock 驱动未启动");
    }
    const uint64_t period_ns = 1000000000ULL / config_.fps;
    const uint64_t deadline = steady_now_ns() + static_cast<uint64_t>(timeout_ms) * 1000000ULL;
    while (steady_now_ns() < next_frame_steady_ns_) {
        const uint64_t now = steady_now_ns();
        const uint64_t remain = next_frame_steady_ns_ > now ? next_frame_steady_ns_ - now : 0;
        if (remain == 0) {
            break;
        }
        const uint64_t sleep_ns = std::min<uint64_t>(remain, 2000000ULL);
        std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_ns));
        if (steady_now_ns() >= deadline && steady_now_ns() < next_frame_steady_ns_) {
            throw CameraError(ErrorCode::Timeout, "Mock 帧等待超时");
        }
    }
    if (steady_now_ns() >= deadline && next_frame_steady_ns_ > deadline) {
        throw CameraError(ErrorCode::Timeout, "Mock 帧等待超时");
    }
    if (config_.mock_disconnect_after_frames != 0 &&
        sequence_ >= config_.mock_disconnect_after_frames) {
        throw CameraError(ErrorCode::DeviceFailure, "Mock 模拟设备断连");
    }

    const uint64_t receive_steady = steady_now_ns();
    const uint64_t receive_system = system_now_ns();
    auto frame = std::make_shared<CameraFrame>();
    frame->provider_instance = instance_;
    frame->sequence = ++sequence_;
    frame->camera_name = config_.name;
    frame->optical_frame = config_.optical_frame;
    frame->calibration_revision = config_.calibration_revision;
    frame->intrinsics.width = config_.color_width;
    frame->intrinsics.height = config_.color_height;
    frame->intrinsics.fx = static_cast<double>(config_.color_width) * 0.9;
    frame->intrinsics.fy = static_cast<double>(config_.color_height) * 0.9;
    frame->intrinsics.cx = static_cast<double>(config_.color_width - 1) * 0.5;
    frame->intrinsics.cy = static_cast<double>(config_.color_height - 1) * 0.5;
    frame->intrinsics.distortion_model = "plumb_bob";
    const size_t pixel_count = static_cast<size_t>(config_.color_width) * config_.color_height;
    frame->rgb.resize(pixel_count * 3);
    frame->depth_m.resize(pixel_count, config_.mock_depth_m);
    const uint8_t phase = static_cast<uint8_t>(frame->sequence & 0xFFU);
    for (uint32_t y = 0; y < config_.color_height; ++y) {
        for (uint32_t x = 0; x < config_.color_width; ++x) {
            const size_t index = static_cast<size_t>(y) * config_.color_width + x;
            frame->rgb[index * 3] = static_cast<uint8_t>((x + phase) & 0xFFU);
            frame->rgb[index * 3 + 1] = static_cast<uint8_t>((y + phase) & 0xFFU);
            frame->rgb[index * 3 + 2] = phase;
        }
    }
    if (config_.mock_invalid_depth_every != 0 &&
        frame->sequence % config_.mock_invalid_depth_every == 0) {
        frame->depth_m[pixel_count / 2] = 0.0F;
    }
    frame->timing = receive_timing(config_, receive_steady, receive_system,
                                   receive_steady / 1000ULL, receive_steady / 1000ULL,
                                   true);
    if (!config_.capture_delay_bound_verified) {
        frame->timing.capture_steady_ns = receive_steady;
        frame->timing.capture_system_ns = receive_system;
        frame->timing.uncertainty_ns = 0;
        frame->timing.trusted = true;
        frame->timing.basis = "mock_clock";
    }
    next_frame_steady_ns_ = receive_steady + period_ns;
    return frame;
}

DeviceDescription MockCameraDriver::description() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return description_;
}

void MockCameraDriver::stop() noexcept {
    running_.store(false);
}

}  // namespace camera_driver
