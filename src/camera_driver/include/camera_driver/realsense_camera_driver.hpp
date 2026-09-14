#pragma once

// Intel RealSense D436 适配器；SDK 依赖严格限制在此模块。
#include "camera_adapter/camera_adapter.hpp"

#include <memory>
#include <mutex>

#ifdef CAMERA_DRIVER_HAS_REALSENSE
#include <librealsense2/rs.hpp>
#endif

namespace camera_driver {

class RealSenseCameraDriver final : public camera_adapter::CameraAdapter {
public:
    void initialize(const camera_adapter::CameraConfig &config) override;
    void start() override;
    std::shared_ptr<camera_adapter::CameraFrame> wait_frame(uint32_t timeout_ms) override;
    camera_adapter::DeviceDescription description() const override;
    void stop() noexcept override;

private:
#ifdef CAMERA_DRIVER_HAS_REALSENSE
    std::shared_ptr<camera_adapter::CameraFrame> make_frame(
        const rs2::frameset &frameset, uint64_t receive_steady_ns,
        uint64_t receive_system_ns);
    rs2::pipeline pipeline_;
    rs2::config pipeline_config_;
    rs2::pipeline_profile pipeline_profile_;
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
