#pragma once

// Gemini335Le 通过 Orbbec SDK v2 接入；未启用 SDK 时保留明确的失败路径。
#include "camera_adapter/camera_adapter.hpp"

#include <memory>
#include <mutex>

#ifdef CAMERA_DRIVER_HAS_ORBBEC
// 当前v0.2.4仍保留v0.2.3布局；因为SDK对象位于公共类中，CMake宏和include必须PUBLIC传播。
#include <libobsensor/ObSensor.hpp>
#endif

namespace camera_driver {

// Orbbec网络/USB设备适配器；对上只输出厂商无关CameraFrame。
class OrbbecCameraDriver final : public camera_adapter::CameraAdapter {
public:
    void initialize(const camera_adapter::CameraConfig &config) override;  // 打开并校验真实设备，准备Profile配置。
    void start() override;                                                // 启动FrameSync/Pipeline并创建新session UUID。
    std::shared_ptr<camera_adapter::CameraFrame> wait_frame(uint32_t timeout_ms) override;  // 等待完整RGB-D帧并复制数据。
    camera_adapter::DeviceDescription description() const override;       // 返回SDK读取到的真实设备信息。
    void stop() noexcept override;                                        // 尽力停止Pipeline，退出路径不抛异常。

private:
#ifdef CAMERA_DRIVER_HAS_ORBBEC
    // 将SDK FrameSet转换为自有内存CameraFrame，并执行内参、深度和时间归一化。
    std::shared_ptr<camera_adapter::CameraFrame> make_frame(
        const std::shared_ptr<ob::FrameSet> &frameset, uint64_t receive_steady_ns,
        uint64_t receive_system_ns);
    std::unique_ptr<ob::Context> context_;                                 // Orbbec SDK Context，仅存在于驱动层。
    std::shared_ptr<ob::Device> device_;                                  // 已打开并核对身份的真实设备。
    std::unique_ptr<ob::Pipeline> pipeline_;                              // RGB-D数据流Pipeline。
    std::shared_ptr<ob::Config> pipeline_config_;                         // 当前Color/Depth/Profile/D2C配置。
#endif
    camera_adapter::CameraConfig config_;                                 // 已验证配置快照。
    camera_adapter::DeviceDescription description_;                       // 真实设备描述。
    camera_adapter::InstanceId instance_{};                               // 当前streaming session UUID；start前全0。
    bool initialized_ = false;                                            // initialize是否成功完成。
    bool running_ = false;                                                // Pipeline是否已成功启动。
    uint64_t sequence_ = 0;                                               // 当前session帧序号。
    mutable std::mutex mutex_;                                            // 串行化生命周期和wait_frame调用。
};

}  // namespace camera_driver
