#pragma once

// Intel RealSense D436 适配器；SDK 依赖严格限制在此模块。
#include "camera_adapter/camera_adapter.hpp"

#include <memory>
#include <mutex>

#ifdef CAMERA_DRIVER_HAS_REALSENSE
// 当前v0.2.4仍保留v0.2.3布局；因此CMake必须向消费者传播相同宏和SDK include目录。
#include <librealsense2/rs.hpp>
#endif

namespace camera_driver {

// RealSense适配器；当前设计为USB序列号绑定、RGB8+Z16、软件D2C。
class RealSenseCameraDriver final : public camera_adapter::CameraAdapter {
public:
    void initialize(const camera_adapter::CameraConfig &config) override;  // 校验配置并准备rs2::config。
    void start() override;                                                // 启动Pipeline、读取真实设备身份并创建session UUID。
    std::shared_ptr<camera_adapter::CameraFrame> wait_frame(uint32_t timeout_ms) override;  // 等待并对齐RGB-D帧。
    camera_adapter::DeviceDescription description() const override;       // 返回SDK读取到的真实设备描述。
    void stop() noexcept override;                                        // 停止Pipeline并释放对齐器。

private:
#ifdef CAMERA_DRIVER_HAS_REALSENSE
    // 将rs2::frameset转换为统一CameraFrame。
    std::shared_ptr<camera_adapter::CameraFrame> make_frame(
        const rs2::frameset &frameset, uint64_t receive_steady_ns,
        uint64_t receive_system_ns);
    rs2::pipeline pipeline_;                                              // RealSense数据流Pipeline。
    rs2::config pipeline_config_;                                         // 设备序列号和流Profile配置。
    rs2::pipeline_profile pipeline_profile_;                              // start后生效的Profile，用于读取设备身份。
    std::unique_ptr<rs2::align> align_to_color_;                          // Depth->Color软件对齐器。
#endif
    camera_adapter::CameraConfig config_;                                 // 已验证配置快照。
    camera_adapter::DeviceDescription description_;                       // 真实设备描述。
    camera_adapter::InstanceId instance_{};                               // 当前streaming session UUID；start前全0。
    bool initialized_ = false;                                            // initialize是否成功完成。
    bool running_ = false;                                                // Pipeline是否运行。
    uint64_t sequence_ = 0;                                               // 当前session帧序号。
    mutable std::mutex mutex_;                                            // 串行化生命周期和取帧。
};

}  // namespace camera_driver
