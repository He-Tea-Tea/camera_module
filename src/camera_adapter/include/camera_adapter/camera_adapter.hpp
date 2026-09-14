#pragma once
// 统一相机数据契约：仅使用标准 C++ 类型，厂商 SDK 不跨越此边界。
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace camera_adapter {

using InstanceId = std::array<uint8_t, 16>;
using SteadyClock = std::chrono::steady_clock;

// 以下错误码仅属于相机内部协议，不占用整机 CommonTypes 的结果码编号。
enum class ErrorCode : uint32_t {
    Ok = 0, NotReady = 1, CameraNotFound = 2, FrameNotFound = 3, StaleFrame = 4,
    InvalidPixel = 5, InvalidDepth = 6, TimeUntrusted = 7, InvalidCalibration = 8,
    DeviceFailure = 9, InvalidConfig = 10, UnsupportedBackend = 11, Timeout = 12,
    Canceled = 13, IoFailure = 14, InvalidRequest = 15, Unsynchronized = 16
};

enum class CameraState : uint8_t {
    Stopped = 0, Connecting = 1, Streaming = 2, Reconnecting = 3, Fault = 4
};

struct CameraError : std::runtime_error {
    ErrorCode code;
    CameraError(ErrorCode error_code, const std::string &message)
        : std::runtime_error(message), code(error_code) {}
};

// 配置属于小脑内部；实例编号由部署配置固定，不能由枚举顺序推导。
struct CameraConfig {
    std::string name = "head_camera";
    std::string backend = "mock";
    std::string model = "Mock RGB-D";
    std::string serial;
    std::string ip_address;
    std::string optical_frame = "head_camera_color_optical_frame";
    std::string color_format = "RGB";
    std::string calibration_revision = "factory-unverified";
    uint16_t component_instance_id = 1;
    uint16_t network_port = 8090;
    uint32_t color_width = 640;
    uint32_t color_height = 480;
    uint32_t depth_width = 640;
    uint32_t depth_height = 480;
    uint32_t fps = 30;
    uint32_t wait_timeout_ms = 200;
    uint32_t disconnect_timeout_ms = 2000;
    uint32_t reconnect_delay_ms = 1000;
    uint32_t cache_capacity = 6;
    uint32_t max_frame_age_ms = 200;
    uint32_t max_pair_delta_ms = 35;
    uint32_t capture_delay_bound_ms = 0;
    bool capture_delay_bound_verified = false;
    bool required_for_auto = true;
    bool allow_unbound_device = false;
    float min_depth_m = 0.2F;
    float max_depth_m = 6.0F;
    float mock_depth_m = 1.0F;
    uint32_t mock_disconnect_after_frames = 0;
    uint32_t mock_invalid_depth_every = 0;
    void validate() const;
};

// 系数顺序遵循 OpenCV/ROS：k1、k2、p1、p2、k3、k4、k5、k6。
struct Intrinsics {
    uint32_t width = 0;
    uint32_t height = 0;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    std::string distortion_model = "plumb_bob";
    std::array<double, 8> distortion{};
    void validate() const;
    std::array<double, 3> deproject(uint32_t u, uint32_t v, double depth_m) const;
};

// 真实设备默认不宣称时间可信；验证过的延迟区间可提供保守年龄上界。
struct FrameTiming {
    uint64_t receive_steady_ns = 0;
    uint64_t capture_steady_ns = 0;
    uint64_t capture_system_ns = 0;
    uint64_t uncertainty_ns = 0;
    uint64_t color_device_us = 0;
    uint64_t depth_device_us = 0;
    uint64_t pair_delta_us = 0;
    bool trusted = false;
    bool pair_synchronized = false;
    std::string basis = "unknown";
};

// 帧离开 SDK 后拥有自己的内存；RGB 为 rgb8，深度为米制 float32，0 表示无效。
struct CameraFrame {
    InstanceId provider_instance{};
    uint64_t sequence = 0;
    std::string camera_name;
    std::string optical_frame;
    std::string calibration_revision;
    Intrinsics intrinsics;
    FrameTiming timing;
    bool depth_aligned_to_color = true;
    std::vector<uint8_t> rgb;
    std::vector<float> depth_m;
    void validate() const;
};

struct DeviceDescription {
    std::string serial;
    std::string model;
    bool imu_supported = false;
    uint32_t active_fps = 0;
};

// 生命周期调用与 wait_frame 由同一个采集线程串行执行，stop 不与读取并发。
class CameraAdapter {
public:
    virtual ~CameraAdapter() = default;
    virtual void initialize(const CameraConfig &config) = 0;
    virtual void start() = 0;
    virtual std::shared_ptr<CameraFrame> wait_frame(uint32_t timeout_ms) = 0;
    virtual DeviceDescription description() const = 0;
    virtual void stop() noexcept = 0;
};

uint64_t steady_now_ns();
uint64_t system_now_ns();
InstanceId new_instance_id();
std::string instance_hex(const InstanceId &instance);
bool is_zero_instance(const InstanceId &instance);
FrameTiming receive_timing(const CameraConfig &config, uint64_t receive_steady_ns,
                           uint64_t receive_system_ns, uint64_t color_device_us,
                           uint64_t depth_device_us, bool comparable_device_clocks);
double frame_age_upper_ms(const FrameTiming &timing, uint64_t now_ns);
std::string error_name(ErrorCode code);

}  // namespace camera_adapter
