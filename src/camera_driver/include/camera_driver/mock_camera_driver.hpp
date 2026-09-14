#pragma once

// 无硬件时提供确定性 RGB-D 数据，用于接口联调和回归测试。
#include "camera_adapter/camera_adapter.hpp"

#include <atomic>
#include <mutex>

namespace camera_driver {

class MockCameraDriver final : public camera_adapter::CameraAdapter {
public:
    void initialize(const camera_adapter::CameraConfig &config) override;
    void start() override;
    std::shared_ptr<camera_adapter::CameraFrame> wait_frame(uint32_t timeout_ms) override;
    camera_adapter::DeviceDescription description() const override;
    void stop() noexcept override;

private:
    camera_adapter::CameraConfig config_;
    camera_adapter::DeviceDescription description_;
    camera_adapter::InstanceId instance_{};
    std::atomic<bool> initialized_{false};
    std::atomic<bool> running_{false};
    uint64_t sequence_ = 0;
    uint64_t next_frame_steady_ns_ = 0;
    mutable std::mutex mutex_;
};

}  // namespace camera_driver
