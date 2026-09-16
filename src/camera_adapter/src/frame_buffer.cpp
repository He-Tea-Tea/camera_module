// 帧缓存和像素反投影查询实现。
#include "camera_adapter/frame_buffer.hpp"

#include <cmath>

namespace camera_adapter {

// 构造时立即复用统一容量校验，避免出现0容量或异常大缓存。
FrameBuffer::FrameBuffer(std::size_t capacity) {
    set_capacity(capacity);
}

// 设置缓存容量；允许1..64，实际CameraConfig当前进一步限制到16。
void FrameBuffer::set_capacity(std::size_t capacity) {
    if (capacity == 0 || capacity > 64) {
        throw CameraError(ErrorCode::InvalidConfig, "帧缓存容量必须为 1..64");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    capacity_ = capacity;
    // 缩容时从最旧帧开始丢弃，保留最新观测。
    while (frames_.size() > capacity_) {
        frames_.pop_front();
    }
}

// 入队前强制验证统一帧契约，任何不完整/不同步帧都不能污染缓存。
void FrameBuffer::push(std::shared_ptr<CameraFrame> frame) {
    if (!frame) {
        throw CameraError(ErrorCode::InvalidRequest, "不能缓存空帧");
    }
    frame->validate();
    std::lock_guard<std::mutex> lock(mutex_);
    frames_.push_back(std::move(frame));
    // 超过容量后始终移除最旧帧。
    while (frames_.size() > capacity_) {
        frames_.pop_front();
    }
}

// 返回最新帧；shared_ptr保证离开锁后帧对象仍安全存在。
std::shared_ptr<const CameraFrame> FrameBuffer::latest() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return frames_.empty() ? nullptr : frames_.back();
}

// 从最新向旧帧查找sequence，符合实时查询通常命中最新窗口的访问模式。
std::shared_ptr<const CameraFrame> FrameBuffer::by_sequence(uint64_t sequence) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto it = frames_.rbegin(); it != frames_.rend(); ++it) {
        if ((*it)->sequence == sequence) {
            return *it;
        }
    }
    return nullptr;
}

// 返回当前缓存大小，用于诊断和测试。
std::size_t FrameBuffer::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return frames_.size();
}

// 清除所有帧；每次重连前必须调用，避免旧session帧进入新会话查询。
void FrameBuffer::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    frames_.clear();
}

// 执行本地像素三维查询并统一处理帧身份、新鲜度、时间可信度和无效深度。
PointSample query_point(const FrameBuffer &buffer, uint32_t u, uint32_t v,
                        uint64_t now_steady_ns, uint32_t max_age_ms,
                        bool require_trusted_time, uint64_t sequence) {
    PointSample result;
    // sequence=0使用最新帧；非0必须精确命中当前有界缓存中的同session序号。
    result.frame = sequence == 0 ? buffer.latest() : buffer.by_sequence(sequence);
    if (!result.frame) {
        result.code = sequence == 0 ? ErrorCode::FrameNotFound : ErrorCode::StaleFrame;
        result.message = sequence == 0 ? "没有可用帧" : "请求的帧不在缓存中";
        return result;
    }
    result.sequence = result.frame->sequence;
    // D2C后的像素坐标必须位于彩色内参定义的图像范围内。
    if (u >= result.frame->intrinsics.width || v >= result.frame->intrinsics.height) {
        result.code = ErrorCode::InvalidPixel;
        result.message = "像素超出图像范围";
        return result;
    }
    // 调用方显式要求可信采集时间时，不允许用接收时间伪装。
    if (require_trusted_time && !result.frame->timing.trusted) {
        result.code = ErrorCode::TimeUntrusted;
        result.message = "调用方要求可信采集时间";
        return result;
    }
    if (result.frame->timing.trusted) {
        // 有可信时间时按采集时间区间计算保守年龄上界。
        result.age_upper_ms = frame_age_upper_ms(result.frame->timing, now_steady_ns);
        if (!std::isfinite(result.age_upper_ms) || result.age_upper_ms > max_age_ms) {
            result.code = ErrorCode::StaleFrame;
            result.message = "帧已过期";
            return result;
        }
    } else {
        // 这里仅限制缓存中的接收时间，绝不把它改写成设备采集时间。
        if (result.frame->timing.receive_steady_ns == 0 ||
            now_steady_ns < result.frame->timing.receive_steady_ns) {
            result.code = ErrorCode::TimeUntrusted;
            result.message = "接收时间不可用";
            return result;
        }
        // 无可信采集时间时仍限制“主机已经接收多久”，防止长期旧缓存被误用。
        result.age_upper_ms = static_cast<double>(
            now_steady_ns - result.frame->timing.receive_steady_ns) / 1e6;
        if (result.age_upper_ms > max_age_ms) {
            result.code = ErrorCode::StaleFrame;
            result.message = "接收缓存帧已过期";
            return result;
        }
    }
    // 计算当前D2C像素在米制depth_m数组中的线性索引。
    const auto index = static_cast<std::size_t>(v) * result.frame->intrinsics.width + u;
    const float depth = result.frame->depth_m[index];
    // v0.2.4驱动已把范围外深度统一置0；这里继续做最终防御式检查。
    if (!std::isfinite(depth) || depth <= 0.0F) {
        result.code = ErrorCode::InvalidDepth;
        result.message = "像素没有有效深度";
        return result;
    }
    try {
        // 使用当前帧内参反投影到frame.optical_frame，单位统一为米。
        result.point_m = result.frame->intrinsics.deproject(u, v, depth);
    } catch (const CameraError &error) {
        result.code = error.code;
        result.message = error.what();
        return result;
    }
    result.code = ErrorCode::Ok;
    result.message = "OK";
    return result;
}

}  // namespace camera_adapter
