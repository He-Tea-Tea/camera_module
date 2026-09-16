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

// 16字节随机实例ID；v0.2.4把它明确作为“单次streaming session”的身份。
using InstanceId = std::array<uint8_t, 16>;
// steady_clock只用于本机持续时间、超时和新鲜度，不用于跨SoC事件时间。
using SteadyClock = std::chrono::steady_clock;

// 以下错误码仅属于相机内部协议，不占用整机 CommonTypes 的结果码编号。
enum class ErrorCode : uint32_t {
    Ok = 0,                    // 操作成功。
    NotReady = 1,              // 驱动或数据流尚未进入可用状态。
    CameraNotFound = 2,        // 指定相机不存在或无法打开。
    FrameNotFound = 3,         // 当前没有可用RGB-D帧。
    StaleFrame = 4,            // 指定帧已离开有界缓存或超过允许年龄。
    InvalidPixel = 5,          // 像素坐标超出当前对齐图像范围。
    InvalidDepth = 6,          // 像素深度为0、非有限值或已被有效范围过滤。
    TimeUntrusted = 7,         // 调用方要求可信采集时间，但当前帧没有可信时间证据。
    InvalidCalibration = 8,    // 内参、畸变、分辨率或对齐状态不满足统一契约。
    DeviceFailure = 9,         // 厂商SDK报告设备级故障。
    InvalidConfig = 10,        // 参数配置违反范围、身份或部署规则。
    UnsupportedBackend = 11,   // 当前构建没有启用所需厂商后端。
    Timeout = 12,              // 等待设备、帧或动作超过约定超时。
    Canceled = 13,             // Action或内部等待被主动取消。
    IoFailure = 14,            // 本地证据文件等I/O操作失败。
    InvalidRequest = 15,       // 请求字段组合不合法。
    Unsynchronized = 16        // RGB与Depth设备时间差超过允许阈值。
};

// 相机私有生命周期状态；不是V1.3公共LifecycleState。
enum class CameraState : uint8_t {
    Stopped = 0,       // 工作线程未运行或已退出。
    Connecting = 1,    // 首次连接/初始化设备。
    Streaming = 2,     // 已持续收到合法RGB-D帧。
    Reconnecting = 3,  // 故障后正在重新建立SDK会话。
    Fault = 4          // 当前检测到故障，等待重连或人工处理。
};

// 带机器可判定错误码的相机内部异常。
struct CameraError : std::runtime_error {
    ErrorCode code;  // 结构化错误码；message只用于诊断。
    CameraError(ErrorCode error_code, const std::string &message)
        : std::runtime_error(message), code(error_code) {}
};

// 配置属于小脑内部；实例编号由部署配置固定，不能由枚举顺序推导。
struct CameraConfig {
    std::string name = "head_camera";                       // 逻辑相机名，作为本地端点和日志主键。
    std::string backend = "mock";                           // mock/orbbec/realsense。
    std::string model = "Mock RGB-D";                       // 期望/展示型号；真实设备启动后会读取SDK型号。
    std::string serial;                                     // 生产设备固定序列号；网络Orbbec也会再次核对。
    std::string ip_address;                                 // 网络相机固定IP；RealSense当前不使用。
    std::string optical_frame = "head_camera_color_optical_frame";  // D2C后3D点所属光学坐标系。
    std::string color_format = "RGB";                       // 统一上层契约当前固定输出RGB8。
    std::string calibration_revision = "factory-unverified";  // 标定制品修订号，未验证时必须显式标识。
    uint16_t component_instance_id = 1;                     // V1.3组件实例号，1..65534且跨重启稳定。
    uint16_t network_port = 8090;                            // Orbbec网络设备端口。
    uint32_t color_width = 640;                              // RGB输出宽度。
    uint32_t color_height = 480;                             // RGB输出高度。
    uint32_t depth_width = 640;                              // 原始/目标Depth宽度。
    uint32_t depth_height = 480;                             // 原始/目标Depth高度。
    uint32_t fps = 30;                                      // 目标帧率。
    uint32_t wait_timeout_ms = 200;                          // 单次SDK等待帧超时。
    uint32_t disconnect_timeout_ms = 2000;                   // 连续无合法帧达到该值后判定会话故障。
    uint32_t reconnect_delay_ms = 1000;                      // 故障后的重连等待时间。
    uint32_t cache_capacity = 6;                             // 有界RGB-D缓存容量。
    uint32_t max_frame_age_ms = 200;                         // 本地查询可接受的最大帧年龄。
    uint32_t max_pair_delta_ms = 35;                         // RGB/Depth设备时间最大差值。
    uint32_t capture_delay_bound_ms = 0;                     // 经外部测量确认的采集到接收延迟上界。
    bool capture_delay_bound_verified = false;               // 未验证延迟时绝不声明可信采集时间。
    bool required_for_auto = true;                           // 预留给整机能力汇聚器的自动模式必需标记。
    bool allow_unbound_device = false;                       // 生产真实设备默认禁止按枚举顺序随意选择。
    float min_depth_m = 0.2F;                                // 有效深度下界；v0.2.4驱动入帧即执行过滤。
    float max_depth_m = 6.0F;                                // 有效深度上界；范围外统一写0。
    float mock_depth_m = 1.0F;                               // Mock确定性深度值。
    uint32_t mock_disconnect_after_frames = 0;               // Mock故障注入：指定帧数后模拟断连，0禁用。
    uint32_t mock_invalid_depth_every = 0;                   // Mock故障注入：每N帧中心点深度置0，0禁用。
    void validate() const;                                   // 启动前集中校验所有静态参数边界。
};

// 系数顺序遵循 OpenCV/ROS：k1、k2、p1、p2、k3、k4、k5、k6。
struct Intrinsics {
    uint32_t width = 0;                                      // 与D2C后RGB/Depth共同像素网格宽度一致。
    uint32_t height = 0;                                     // 与D2C后RGB/Depth共同像素网格高度一致。
    double fx = 0.0;                                         // x方向焦距，单位像素。
    double fy = 0.0;                                         // y方向焦距，单位像素。
    double cx = 0.0;                                         // 主点x坐标，单位像素。
    double cy = 0.0;                                         // 主点y坐标，单位像素。
    std::string distortion_model = "plumb_bob";             // 当前统一契约支持的畸变模型名称。
    std::array<double, 8> distortion{};                      // k1,k2,p1,p2,k3,k4,k5,k6。
    void validate() const;                                   // 校验内参和畸变参数是否可用于反投影。
    std::array<double, 3> deproject(uint32_t u, uint32_t v, double depth_m) const;  // 像素+深度反投影到光学坐标系。
};

// 真实设备默认不宣称时间可信；验证过的延迟区间可提供保守年龄上界。
struct FrameTiming {
    uint64_t receive_steady_ns = 0;                          // SDK返回完整帧组后的本机steady接收时间。
    uint64_t capture_steady_ns = 0;                          // 仅在延迟上界验证后估计的本机steady采集时间。
    uint64_t capture_system_ns = 0;                          // 仅在延迟上界验证后给审计/ROS Time使用的系统时间。
    uint64_t uncertainty_ns = 0;                             // 采集时间估计的不确定度上界。
    uint64_t color_device_us = 0;                            // Color设备时间戳，单位微秒。
    uint64_t depth_device_us = 0;                            // Depth设备时间戳，单位微秒。
    uint64_t pair_delta_us = 0;                              // RGB/Depth设备时间差，单位微秒。
    bool trusted = false;                                    // 是否允许把capture_*作为可信采集时间使用。
    bool pair_synchronized = false;                          // RGB/Depth是否通过max_pair_delta_ms门控。
    std::string basis = "unknown";                          // 时间可信度依据，例如mock_clock或verified_receive_interval。
};

// 帧离开 SDK 后拥有自己的内存；RGB 为 rgb8，深度为米制 float32，0 表示无效。
struct CameraFrame {
    InstanceId provider_instance{};                          // 当前streaming session UUID；重连后必须变化。
    uint64_t sequence = 0;                                   // 当前session内严格递增的帧序号，从1开始。
    std::string camera_name;                                 // 逻辑相机名称。
    std::string optical_frame;                               // 三维坐标所属相机光学坐标系。
    std::string calibration_revision;                        // 生成该帧所依据的标定修订号。
    Intrinsics intrinsics;                                   // 与当前D2C输出网格匹配的彩色内参。
    FrameTiming timing;                                      // 接收、设备时间和可信度信息。
    bool depth_aligned_to_color = true;                      // 深度是否已经对齐到彩色像素网格。
    std::vector<uint8_t> rgb;                                // 自有RGB8内存，大小=width*height*3。
    std::vector<float> depth_m;                              // 自有米制深度，大小=width*height，0表示无效。
    void validate() const;                                   // 缓存入队前验证统一帧契约。
};

// 真实设备身份和当前启用能力的简要描述。
struct DeviceDescription {
    std::string serial;                                      // SDK读取到的真实序列号。
    std::string model;                                       // SDK读取到的真实设备型号。
    bool imu_supported = false;                              // 当前RGB-D后端是否声明IMU能力。
    uint32_t active_fps = 0;                                 // 当前启用的RGB-D目标帧率。
};

// 生命周期调用与 wait_frame 由同一个采集线程串行执行，stop 不与读取并发。
class CameraAdapter {
public:
    virtual ~CameraAdapter() = default;
    virtual void initialize(const CameraConfig &config) = 0;  // 校验配置并准备厂商对象，不代表已产生streaming session。
    virtual void start() = 0;                                 // 真正启动数据流；成功后生成新的session UUID。
    virtual std::shared_ptr<CameraFrame> wait_frame(uint32_t timeout_ms) = 0;  // 等待并返回一个完整统一RGB-D帧。
    virtual DeviceDescription description() const = 0;        // 返回已打开设备的真实描述。
    virtual void stop() noexcept = 0;                          // 尽力停止数据流；析构/退出路径不得抛异常。
};

uint64_t steady_now_ns();                                    // 获取本机steady_clock纳秒。
uint64_t system_now_ns();                                    // 获取本机system_clock纳秒。
InstanceId new_instance_id();                                // 生成随机非零UUIDv4风格16字节session ID。
std::string instance_hex(const InstanceId &instance);        // 将16字节实例ID转换为32位十六进制字符串。
bool is_zero_instance(const InstanceId &instance);            // 判断实例ID是否全0。
FrameTiming receive_timing(const CameraConfig &config, uint64_t receive_steady_ns,
                           uint64_t receive_system_ns, uint64_t color_device_us,
                           uint64_t depth_device_us, bool comparable_device_clocks);  // 生成统一时间质量记录。
double frame_age_upper_ms(const FrameTiming &timing, uint64_t now_ns);  // 计算可信时间条件下的保守帧年龄上界。
std::string error_name(ErrorCode code);                       // 将内部错误码转换为稳定诊断名称。

}  // namespace camera_adapter
