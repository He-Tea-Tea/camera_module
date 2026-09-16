// 确定性 Mock 驱动：模拟采集节拍、无效深度和断连。
#include "camera_driver/mock_camera_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace camera_driver {
using namespace camera_adapter;

// initialize只保存配置和设备描述；v0.2.4不在此阶段生成streaming session UUID。
void MockCameraDriver::initialize(const CameraConfig &config) {
    config.validate();
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
    instance_ = {};                 // 尚未start，明确保持全0实例。
    sequence_ = 0;                  // 尚未start，帧序号清零。
    next_frame_steady_ns_ = 0;      // 下一帧节拍由start初始化。
    description_.serial = config.serial.empty() ? "MOCK-" + config.name : config.serial;
    description_.model = config.model;
    description_.imu_supported = false;
    description_.active_fps = config.fps;
    initialized_.store(true);
    running_.store(false);
}

// 每次start成功都代表一个新的数据流会话，因此生成新的session UUID并重置sequence。
void MockCameraDriver::start() {
    if (!initialized_.load()) {
        throw CameraError(ErrorCode::NotReady, "Mock 驱动尚未初始化");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    instance_ = new_instance_id();  // 新streaming session身份。
    sequence_ = 0;                  // 新session下一帧从1开始。
    next_frame_steady_ns_ = steady_now_ns();  // 第一帧允许立即产生。
    running_.store(true);
}

// 按配置fps生成一帧确定性RGB-D；支持超时、断连和无效深度故障注入。
std::shared_ptr<CameraFrame> MockCameraDriver::wait_frame(uint32_t timeout_ms) {
    if (!running_.load()) {
        throw CameraError(ErrorCode::NotReady, "Mock 驱动未启动");
    }
    // 计算目标帧周期和本次调用绝对截止时间。
    const uint64_t period_ns = 1000000000ULL / config_.fps;
    const uint64_t deadline = steady_now_ns() + static_cast<uint64_t>(timeout_ms) * 1000000ULL;
    // 以最多2ms小片段睡眠模拟阻塞取帧，同时保持超时响应。
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
    // 如果下一目标帧本身已经超出deadline，直接报告Timeout。
    if (steady_now_ns() >= deadline && next_frame_steady_ns_ > deadline) {
        throw CameraError(ErrorCode::Timeout, "Mock 帧等待超时");
    }
    // 故障注入：达到指定帧数后持续模拟设备断连，触发registry重连路径。
    if (config_.mock_disconnect_after_frames != 0 &&
        sequence_ >= config_.mock_disconnect_after_frames) {
        throw CameraError(ErrorCode::DeviceFailure, "Mock 模拟设备断连");
    }

    // 接收时间在“帧已经可用”之后采样，与真实SDK后端保持一致。
    const uint64_t receive_steady = steady_now_ns();
    const uint64_t receive_system = system_now_ns();
    auto frame = std::make_shared<CameraFrame>();
    // 帧身份由当前session UUID+session内sequence共同定义。
    frame->provider_instance = instance_;
    frame->sequence = ++sequence_;
    frame->camera_name = config_.name;
    frame->optical_frame = config_.optical_frame;
    frame->calibration_revision = config_.calibration_revision;
    // 生成稳定可用的模拟内参。
    frame->intrinsics.width = config_.color_width;
    frame->intrinsics.height = config_.color_height;
    frame->intrinsics.fx = static_cast<double>(config_.color_width) * 0.9;
    frame->intrinsics.fy = static_cast<double>(config_.color_height) * 0.9;
    frame->intrinsics.cx = static_cast<double>(config_.color_width - 1) * 0.5;
    frame->intrinsics.cy = static_cast<double>(config_.color_height - 1) * 0.5;
    frame->intrinsics.distortion_model = "plumb_bob";
    // 为RGB和Depth分配完全自有的内存。
    const size_t pixel_count = static_cast<size_t>(config_.color_width) * config_.color_height;
    frame->rgb.resize(pixel_count * 3);
    frame->depth_m.resize(pixel_count, config_.mock_depth_m);
    // 使用sequence相位生成可观察变化的RGB测试图案。
    const uint8_t phase = static_cast<uint8_t>(frame->sequence & 0xFFU);
    for (uint32_t y = 0; y < config_.color_height; ++y) {
        for (uint32_t x = 0; x < config_.color_width; ++x) {
            const size_t index = static_cast<size_t>(y) * config_.color_width + x;
            frame->rgb[index * 3] = static_cast<uint8_t>((x + phase) & 0xFFU);
            frame->rgb[index * 3 + 1] = static_cast<uint8_t>((y + phase) & 0xFFU);
            frame->rgb[index * 3 + 2] = phase;
        }
    }
    // 故障注入：按配置周期把中心深度置0，用于验证INVALID_DEPTH。
    if (config_.mock_invalid_depth_every != 0 &&
        frame->sequence % config_.mock_invalid_depth_every == 0) {
        frame->depth_m[pixel_count / 2] = 0.0F;
    }
    // Mock的Color/Depth使用同一个steady时钟，天然可比较且pair delta为0。
    frame->timing = receive_timing(config_, receive_steady, receive_system,
                                   receive_steady / 1000ULL, receive_steady / 1000ULL,
                                   true);
    // Mock是完全受控的同机测试源，因此未配置真实延迟上界时仍可把模拟采集时间视为可信。
    if (!config_.capture_delay_bound_verified) {
        frame->timing.capture_steady_ns = receive_steady;
        frame->timing.capture_system_ns = receive_system;
        frame->timing.uncertainty_ns = 0;
        frame->timing.trusted = true;
        frame->timing.basis = "mock_clock";
    }
    // 下一目标帧从本次实际接收时刻继续推进一个周期，避免测试长时间积累追帧。
    next_frame_steady_ns_ = receive_steady + period_ns;
    return frame;
}

// 返回initialize阶段构造的Mock设备描述。
DeviceDescription MockCameraDriver::description() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return description_;
}

// Mock停止只需要关闭running标记；该函数必须noexcept。
void MockCameraDriver::stop() noexcept {
    running_.store(false);
}

}  // namespace camera_driver
