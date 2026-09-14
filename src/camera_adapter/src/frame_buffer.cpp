// 帧缓存和像素反投影查询实现。
#include "camera_adapter/frame_buffer.hpp"

#include <cmath>

namespace camera_adapter {

FrameBuffer::FrameBuffer(std::size_t capacity) {
    set_capacity(capacity);
}

void FrameBuffer::set_capacity(std::size_t capacity) {
    if (capacity == 0 || capacity > 64) {
        throw CameraError(ErrorCode::InvalidConfig, "帧缓存容量必须为 1..64");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    capacity_ = capacity;
    while (frames_.size() > capacity_) {
        frames_.pop_front();
    }
}

void FrameBuffer::push(std::shared_ptr<CameraFrame> frame) {
    if (!frame) {
        throw CameraError(ErrorCode::InvalidRequest, "不能缓存空帧");
    }
    frame->validate();
    std::lock_guard<std::mutex> lock(mutex_);
    frames_.push_back(std::move(frame));
    while (frames_.size() > capacity_) {
        frames_.pop_front();
    }
}

std::shared_ptr<const CameraFrame> FrameBuffer::latest() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return frames_.empty() ? nullptr : frames_.back();
}

std::shared_ptr<const CameraFrame> FrameBuffer::by_sequence(uint64_t sequence) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto it = frames_.rbegin(); it != frames_.rend(); ++it) {
        if ((*it)->sequence == sequence) {
            return *it;
        }
    }
    return nullptr;
}

std::size_t FrameBuffer::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return frames_.size();
}

void FrameBuffer::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    frames_.clear();
}

PointSample query_point(const FrameBuffer &buffer, uint32_t u, uint32_t v,
                        uint64_t now_steady_ns, uint32_t max_age_ms,
                        bool require_trusted_time, uint64_t sequence) {
    PointSample result;
    result.frame = sequence == 0 ? buffer.latest() : buffer.by_sequence(sequence);
    if (!result.frame) {
        result.code = sequence == 0 ? ErrorCode::FrameNotFound : ErrorCode::StaleFrame;
        result.message = sequence == 0 ? "没有可用帧" : "请求的帧不在缓存中";
        return result;
    }
    result.sequence = result.frame->sequence;
    if (u >= result.frame->intrinsics.width || v >= result.frame->intrinsics.height) {
        result.code = ErrorCode::InvalidPixel;
        result.message = "像素超出图像范围";
        return result;
    }
    if (require_trusted_time && !result.frame->timing.trusted) {
        result.code = ErrorCode::TimeUntrusted;
        result.message = "调用方要求可信采集时间";
        return result;
    }
    if (result.frame->timing.trusted) {
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
        result.age_upper_ms = static_cast<double>(
            now_steady_ns - result.frame->timing.receive_steady_ns) / 1e6;
        if (result.age_upper_ms > max_age_ms) {
            result.code = ErrorCode::StaleFrame;
            result.message = "接收缓存帧已过期";
            return result;
        }
    }
    const auto index = static_cast<std::size_t>(v) * result.frame->intrinsics.width + u;
    const float depth = result.frame->depth_m[index];
    if (!std::isfinite(depth) || depth <= 0.0F) {
        result.code = ErrorCode::InvalidDepth;
        result.message = "像素没有有效深度";
        return result;
    }
    try {
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
