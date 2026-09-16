// 通用参数校验、时间区间和畸变反投影实现。
#include "camera_adapter/camera_adapter.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <random>
#include <regex>
#include <sstream>

namespace camera_adapter {

uint64_t steady_now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        SteadyClock::now().time_since_epoch()).count();
}

uint64_t system_now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

InstanceId new_instance_id() {
    // 每次重新连接生成新实例，防止旧帧编号误匹配新会话。
    std::random_device random;
    InstanceId result{};
    for (auto &value : result) {
        value = static_cast<uint8_t>(random());
    }
    result[6] = static_cast<uint8_t>((result[6] & 0x0F) | 0x40);
    result[8] = static_cast<uint8_t>((result[8] & 0x3F) | 0x80);
    return result;
}

std::string instance_hex(const InstanceId &instance) {
    std::ostringstream stream;
    stream << std::hex << std::setfill('0');
    for (auto value : instance) {
        stream << std::setw(2) << static_cast<unsigned>(value);
    }
    return stream.str();
}

bool is_zero_instance(const InstanceId &instance) {
    return std::all_of(instance.begin(), instance.end(), [](uint8_t value) { return value == 0; });
}

void CameraConfig::validate() const {
    const std::regex identifier("[A-Za-z][A-Za-z0-9_]*");
    if (!std::regex_match(name, identifier) || name.size() > 48 ||
        !std::regex_match(optical_frame, identifier) || optical_frame.size() > 96) {
        throw CameraError(ErrorCode::InvalidConfig, "相机名称或光学坐标系名称非法");
    }
    if (backend != "mock" && backend != "orbbec" && backend != "realsense") {
        throw CameraError(ErrorCode::InvalidConfig, "backend 只支持 mock、orbbec、realsense");
    }
    if (component_instance_id == 0 || component_instance_id == 65535) {
        throw CameraError(ErrorCode::InvalidConfig, "组件实例编号必须为 1..65534");
    }
    if (color_width < 8 || color_height < 8 || depth_width < 8 || depth_height < 8 ||
        color_width > 4096 || color_height > 2160 || depth_width > 4096 || depth_height > 2160 ||
        fps == 0 || fps > 120 || cache_capacity == 0 || cache_capacity > 16) {
        throw CameraError(ErrorCode::InvalidConfig, "图像尺寸、帧率或缓存容量超出允许范围");
    }
    if (static_cast<uint64_t>(color_width) * color_height * 7 * (cache_capacity + 3) > 268435456ULL) {
        throw CameraError(ErrorCode::InvalidConfig, "单相机预计帧内存超过 256 MiB，请降低分辨率或缓存容量");
    }
    if (wait_timeout_ms < 10 || wait_timeout_ms > 1000 || reconnect_delay_ms < 50 ||
        reconnect_delay_ms > 60000 || disconnect_timeout_ms < wait_timeout_ms ||
        disconnect_timeout_ms > 60000 || max_frame_age_ms == 0 || max_frame_age_ms > 60000 ||
        max_pair_delta_ms > 1000 || capture_delay_bound_ms > 60000) {
        throw CameraError(ErrorCode::InvalidConfig, "超时、同步容差或年龄限制非法");
    }
    if (!std::isfinite(min_depth_m) || !std::isfinite(max_depth_m) ||
        min_depth_m <= 0.0F || max_depth_m <= min_depth_m || max_depth_m > 100.0F ||
        !std::isfinite(mock_depth_m) || mock_depth_m < min_depth_m || mock_depth_m > max_depth_m) {
        throw CameraError(ErrorCode::InvalidConfig, "深度范围或模拟深度非法");
    }
    if (capture_delay_bound_verified && capture_delay_bound_ms == 0) {
        throw CameraError(ErrorCode::InvalidConfig, "已验证的采集延迟上界必须大于零");
    }
    if (serial.size() > 96 || ip_address.size() > 64 || model.size() > 96 ||
        calibration_revision.empty() || calibration_revision.size() > 96) {
        throw CameraError(ErrorCode::InvalidConfig, "设备描述或标定修订长度非法");
    }
    if (backend != "mock" && !allow_unbound_device && serial.empty() && ip_address.empty()) {
        throw CameraError(ErrorCode::InvalidConfig, "真实设备必须绑定序列号或 IP；单机调试可显式允许未绑定");
    }
    if (backend == "realsense" && !ip_address.empty()) {
        throw CameraError(ErrorCode::InvalidConfig, "本 RealSense 后端仅支持按 SDK 序列号选择设备");
    }
    if (backend == "orbbec" && !ip_address.empty() && network_port == 0) {
        throw CameraError(ErrorCode::InvalidConfig, "Orbbec 网络设备端口必须为 1..65535");
    }
}

void Intrinsics::validate() const {
    if (width == 0 || height == 0 || !std::isfinite(fx) || !std::isfinite(fy) ||
        !std::isfinite(cx) || !std::isfinite(cy) || fx <= 0.0 || fy <= 0.0 ||
        !std::all_of(distortion.begin(), distortion.end(), [](double value) { return std::isfinite(value); })) {
        throw CameraError(ErrorCode::InvalidCalibration, "相机内参包含非法值");
    }
    if (distortion_model != "plumb_bob" && distortion_model != "rational_polynomial" &&
        distortion_model != "inverse_brown_conrady" && distortion_model != "modified_brown_conrady") {
        throw CameraError(ErrorCode::InvalidCalibration, "当前标定的畸变模型不受支持");
    }
}

std::array<double, 3> Intrinsics::deproject(uint32_t u, uint32_t v, double depth) const {
    validate();
    if (u >= width || v >= height) {
        throw CameraError(ErrorCode::InvalidPixel, "像素超出图像范围");
    }
    if (!std::isfinite(depth) || depth <= 0.0) {
        throw CameraError(ErrorCode::InvalidDepth, "无有效深度");
    }
    const double target_x = (static_cast<double>(u) - cx) / fx;
    const double target_y = (static_cast<double>(v) - cy) / fy;
    const auto distort = [this](double x, double y) {
        const auto &d = distortion;
        const double r2 = x * x + y * y;
        const double denominator = 1.0 + d[5] * r2 + d[6] * r2 * r2 + d[7] * r2 * r2 * r2;
        if (std::abs(denominator) < 1e-12) {
            throw CameraError(ErrorCode::InvalidCalibration, "畸变分母接近零");
        }
        const double radial = (1.0 + d[0] * r2 + d[1] * r2 * r2 + d[4] * r2 * r2 * r2) / denominator;
        double tx = x;
        double ty = y;
        if (distortion_model == "modified_brown_conrady") {
            tx *= radial;
            ty *= radial;
        }
        return std::array<double, 2>{
            x * radial + 2.0 * d[2] * tx * ty + d[3] * (r2 + 2.0 * tx * tx),
            y * radial + 2.0 * d[3] * tx * ty + d[2] * (r2 + 2.0 * ty * ty)};
    };
    double x = target_x;
    double y = target_y;
    // 牛顿迭代反解各支持模型，残差不收敛时拒绝返回伪精确坐标。
    for (int iteration = 0; iteration < 20; ++iteration) {
        const auto current = distort(x, y);
        const double error_x = current[0] - target_x;
        const double error_y = current[1] - target_y;
        if (std::hypot(error_x, error_y) < 1e-10) {
            return {x * depth, y * depth, depth};
        }
        constexpr double delta = 1e-6;
        const auto dx = distort(x + delta, y);
        const auto dy = distort(x, y + delta);
        const double j00 = (dx[0] - current[0]) / delta;
        const double j10 = (dx[1] - current[1]) / delta;
        const double j01 = (dy[0] - current[0]) / delta;
        const double j11 = (dy[1] - current[1]) / delta;
        const double determinant = j00 * j11 - j01 * j10;
        if (!std::isfinite(determinant) || std::abs(determinant) < 1e-12) {
            break;
        }
        x -= (j11 * error_x - j01 * error_y) / determinant;
        y -= (-j10 * error_x + j00 * error_y) / determinant;
        if (!std::isfinite(x) || !std::isfinite(y) || std::hypot(x, y) > 100.0) {
            break;
        }
    }
    throw CameraError(ErrorCode::InvalidCalibration, "畸变反解未收敛");
}

void CameraFrame::validate() const {
    if (is_zero_instance(provider_instance) || sequence == 0 || camera_name.empty() ||
        optical_frame.empty() || calibration_revision.empty()) {
        throw CameraError(ErrorCode::InvalidCalibration, "帧身份或标定修订为空");
    }
    intrinsics.validate();
    const size_t pixels = static_cast<size_t>(intrinsics.width) * intrinsics.height;
    if (rgb.size() != pixels * 3 || depth_m.size() != pixels || !depth_aligned_to_color) {
        throw CameraError(ErrorCode::InvalidCalibration, "RGB-D 尺寸或对齐状态不一致");
    }
    if (!timing.pair_synchronized) {
        throw CameraError(ErrorCode::Unsynchronized, "RGB 与 Depth 的设备采集时间不匹配");
    }
}

FrameTiming receive_timing(const CameraConfig &config, uint64_t receipt, uint64_t system,
                           uint64_t color_us, uint64_t depth_us, bool comparable) {
    FrameTiming timing;
    timing.receive_steady_ns = receipt;
    timing.color_device_us = color_us;
    timing.depth_device_us = depth_us;
    timing.pair_delta_us = color_us > depth_us ? color_us - depth_us : depth_us - color_us;
    timing.pair_synchronized = comparable && timing.pair_delta_us <= config.max_pair_delta_ms * 1000ULL;
    const uint64_t bound = config.capture_delay_bound_ms * 1000000ULL;
    // 区间必须覆盖整组帧中较早的采集时刻；由外部测量确认，默认不假定成立。
    if (config.capture_delay_bound_verified && bound > 0 && receipt > bound && system > bound) {
        timing.capture_steady_ns = receipt - bound / 2;
        timing.capture_system_ns = system - bound / 2;
        timing.uncertainty_ns = bound - bound / 2;
        timing.trusted = true;
        timing.basis = "verified_receive_interval";
    }
    return timing;
}

double frame_age_upper_ms(const FrameTiming &timing, uint64_t now) {
    if (!timing.trusted || timing.capture_steady_ns == 0 || timing.capture_steady_ns > now) {
        return std::numeric_limits<double>::infinity();
    }
    return static_cast<double>(now - timing.capture_steady_ns) / 1e6 +
           static_cast<double>(timing.uncertainty_ns) / 1e6;
}

std::string error_name(ErrorCode code) {
    static const std::array<const char *, 17> names = {
        "OK", "NOT_READY", "CAMERA_NOT_FOUND", "FRAME_NOT_FOUND", "STALE_FRAME",
        "INVALID_PIXEL", "INVALID_DEPTH", "TIME_UNTRUSTED", "INVALID_CALIBRATION",
        "DEVICE_FAILURE", "INVALID_CONFIG", "UNSUPPORTED_BACKEND", "TIMEOUT",
        "CANCELED", "IO_FAILURE", "INVALID_REQUEST", "UNSYNCHRONIZED"};
    const auto index = static_cast<size_t>(code);
    return index < names.size() ? names[index] : "UNKNOWN";
}

}  // namespace camera_adapter
