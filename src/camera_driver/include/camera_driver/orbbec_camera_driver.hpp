#pragma once

// Gemini335Le 通过 Orbbec SDK v2 接入；未启用 SDK 时保留明确的失败路径。
#include "camera_adapter/camera_adapter.hpp"

#include <memory>
#include <mutex>

#ifdef CAMERA_DRIVER_HAS_ORBBEC
#include <libobsensor/ObSensor.hpp>
#endif

namespace camera_driver {

class OrbbecCameraDriver final : public camera_adapter::CameraAdapter {
public:
    void initialize(const camera_adapter::CameraConfig &config) override;
    void start() override;
    std::shared_ptr<camera_adapter::CameraFrame> wait_frame(uint32_t timeout_ms) override;
    camera_adapter::DeviceDescription description() const override;
    void stop() noexcept override;

private:
#ifdef CAMERA_DRIVER_HAS_ORBBEC
    std::shared_ptr<camera_adapter::CameraFrame> make_frame(
        const std::shared_ptr<ob::FrameSet> &frameset, uint64_t receive_steady_ns,
        uint64_t receive_system_ns);
    std::unique_ptr<ob::Context> context_;
    std::shared_ptr<ob::Device> device_;
    std::unique_ptr<ob::Pipeline> pipeline_;
    std::shared_ptr<ob::Config> pipeline_config_;
#endif
    camera_adapter::CameraConfig config_;
    camera_adapter::DeviceDescription description_;
    camera_adapter::InstanceId instance_{};
    bool initialized_ = false;
    bool running_ = false;
    uint64_t sequence_ = 0;
    mutable std::mutex mutex_;
};

}  // namespace camera_driver
