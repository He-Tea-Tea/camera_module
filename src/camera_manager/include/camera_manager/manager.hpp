#pragma once

// 相机管理器只编排生命周期和本地缓存，不把厂商 SDK 暴露给上层。
#include "camera_adapter/camera_adapter.hpp"
#include "camera_adapter/frame_buffer.hpp"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace camera_manager {

// 单相机本地能力摘要；后续由整机capability aggregator转换为V1.3公共强类型能力。
struct CameraCapability {
    uint16_t component_instance_id = 0;                    // V1.3稳定组件实例号。
    uint8_t component_kind = 7;                            // COMPONENT_CAMERA=7。
    uint8_t device_id = 255;                               // 相机无公共DeviceState，使用DEVICE_NONE=255。
    std::string camera_name;                               // 小脑本地逻辑名称。
    std::string model;                                     // SDK真实型号优先，未连接时回退配置型号。
    std::string serial;                                    // SDK真实序列号优先，未连接时回退配置序列号。
    std::string optical_frame;                             // 本地三维坐标所属光学坐标系。
    std::string calibration_revision;                      // 当前标定revision。
    bool local_camera_available = false;                   // 只有STREAMING时为true。
    bool capture_delay_bound_verified = false;             // 是否验证采集到接收延迟上界。
    uint32_t capture_delay_bound_ms = 0;                   // 已验证延迟上界。
    uint32_t max_frame_age_ms = 0;                         // 本地查询最大帧年龄。
    uint32_t max_pair_delta_ms = 0;                        // RGB-D最大设备时间差。
    std::string closed_loop_control_location = "small_brain";  // 私有文本语义对应CLOSED_LOOP_CEREBELLUM。
    std::string hw_ptp_level = "none";                    // 私有文本语义对应HW_PTP_NONE。
};

// 单相机运行状态快照；v0.2.4增加streaming session UUID，解决重连后sequence复用歧义。
struct CameraStatus {
    camera_adapter::CameraState state = camera_adapter::CameraState::Stopped;  // 私有生命周期状态。
    camera_adapter::InstanceId frame_session_uuid{};          // 当前/最近合法streaming session UUID。
    uint64_t frame_count = 0;                                 // 当前session已经接收的合法帧数。
    uint64_t last_sequence = 0;                               // 当前session最近帧序号。
    uint64_t last_receive_steady_ns = 0;                      // 最近帧主机接收steady时间。
    uint64_t last_capture_system_ns = 0;                      // 最近可信采集系统时间；不可信时0。
    uint64_t state_since_steady_ns = 0;                       // 当前连接/故障状态起始steady时间。
    bool time_trusted = false;                                // 最近帧采集时间是否可信。
    bool pair_synchronized = false;                           // 最近RGB-D pair是否通过同步门控。
    std::string last_error;                                   // 最近错误诊断；健康时为空。
    camera_adapter::ErrorCode last_error_code = camera_adapter::ErrorCode::Ok;  // 最近内部错误码。
};

// 将CameraStatus映射成简短健康标签，仅用于日志/测试，不属于公共协议枚举。
const char *health_label(const CameraStatus &status);

// 多相机Registry：每台相机一个独立Record、缓存、后台工作线程和自动重连循环。
class CameraRegistry {
public:
    explicit CameraRegistry(std::vector<camera_adapter::CameraConfig> configs);  // 校验配置并创建Record。
    ~CameraRegistry();                                           // 析构时调用stop。

    void initialize();                                           // 标记Registry可启动；不在调用线程连接硬件。
    void start();                                                // 为每台相机启动后台工作线程。
    void stop() noexcept;                                        // 请求所有线程退出并尽力join。
    bool started() const;                                        // Registry是否处于启动状态。

    std::vector<CameraCapability> capabilities() const;          // 返回所有相机能力快照。
    CameraStatus status(const std::string &camera_name) const;   // 返回指定相机状态快照。
    std::shared_ptr<const camera_adapter::CameraFrame> latest_frame(
        const std::string &camera_name) const;                    // 返回指定相机最新缓存帧。
    camera_adapter::PointSample query_point(
        const std::string &camera_name, uint32_t u, uint32_t v,
        bool require_trusted_time, uint64_t sequence = 0) const;  // 本地像素三维查询。

private:
    // 每台相机独立的运行记录；shared_ptr保证极端SDK阻塞线程detach后对象不会提前析构。
    struct Record {
        camera_adapter::CameraConfig config;                      // 当前相机固定配置。
        camera_adapter::FrameBuffer buffer;                       // 当前session有界RGB-D缓存。
        mutable std::mutex mutex;                                 // 保护description/status。
        std::mutex finish_mutex;                                  // 保护线程完成条件等待。
        std::condition_variable finish_condition;                 // stop等待工作线程退出。
        std::thread worker;                                       // 相机生命周期和取帧线程。
        std::atomic<bool> stop_requested{false};                  // 请求线程尽快退出。
        std::atomic<bool> worker_finished{true};                  // 工作线程是否已经完成。
        camera_adapter::DeviceDescription description;            // 当前/最近真实设备描述。
        CameraStatus status;                                      // 当前状态快照。

        explicit Record(camera_adapter::CameraConfig value)
            : config(std::move(value)), buffer(config.cache_capacity) {}
    };

    static std::unique_ptr<camera_adapter::CameraAdapter> make_adapter(
        const camera_adapter::CameraConfig &config);               // 根据backend创建对应驱动。
    static void set_error(Record &record, const camera_adapter::CameraError &error);  // 原子更新故障状态。
    static bool wait_interruptible(Record &record, uint32_t delay_ms);  // 可被stop打断的重连等待。
    static void worker_loop(const std::shared_ptr<Record> &record);    // 单相机连接/采集/重连主循环。
    Record &get_record(const std::string &camera_name) const;          // 按逻辑名定位Record，不存在抛CameraNotFound。

    std::unordered_map<std::string, std::shared_ptr<Record>> records_; // camera_name到Record映射。
    mutable std::mutex mutex_;                                        // 保护Registry整体start/stop/capabilities遍历。
    std::atomic<bool> initialized_{false};                             // initialize是否调用。
    std::atomic<bool> started_{false};                                 // Registry是否已经启动工作线程。
};

}  // namespace camera_manager
