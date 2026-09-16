#pragma once

// 无硬件时提供确定性 RGB-D 数据，用于接口联调和回归测试。
#include "camera_adapter/camera_adapter.hpp"

#include <atomic>
#include <mutex>

namespace camera_driver {

// Mock驱动遵循与真实设备完全相同的CameraAdapter生命周期和session UUID语义。
class MockCameraDriver final : public camera_adapter::CameraAdapter {
public:
    void initialize(const camera_adapter::CameraConfig &config) override;  // 保存并校验Mock配置，不创建streaming session。
    void start() override;                                                // 启动新session并把sequence重置为0。
    std::shared_ptr<camera_adapter::CameraFrame> wait_frame(uint32_t timeout_ms) override;  // 按fps生成确定性RGB-D帧。
    camera_adapter::DeviceDescription description() const override;       // 返回Mock设备身份。
    void stop() noexcept override;                                        // 停止Mock生成，不抛异常。

private:
    camera_adapter::CameraConfig config_;                                 // 当前后端配置快照。
    camera_adapter::DeviceDescription description_;                       // Mock设备描述。
    camera_adapter::InstanceId instance_{};                               // 当前streaming session UUID；未start时全0。
    std::atomic<bool> initialized_{false};                                // initialize是否成功完成。
    std::atomic<bool> running_{false};                                    // 当前是否允许wait_frame产生帧。
    uint64_t sequence_ = 0;                                               // 当前session帧序号。
    uint64_t next_frame_steady_ns_ = 0;                                   // 下一帧计划steady时间，用于模拟fps。
    mutable std::mutex mutex_;                                            // 保护配置、描述和session初始化。
};

}  // namespace camera_driver
