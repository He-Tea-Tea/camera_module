#pragma once

// 有界帧缓存只保存已经脱离厂商 SDK 的自有内存帧。
#include "camera_adapter/camera_adapter.hpp"

#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>

namespace camera_adapter {

struct PointSample {
    ErrorCode code = ErrorCode::NotReady;
    std::string message;
    std::shared_ptr<const CameraFrame> frame;
    uint64_t sequence = 0;
    std::array<double, 3> point_m{};
    double age_upper_ms = 0.0;
};

class FrameBuffer {
public:
    explicit FrameBuffer(std::size_t capacity = 6);

    void set_capacity(std::size_t capacity);
    void push(std::shared_ptr<CameraFrame> frame);
    std::shared_ptr<const CameraFrame> latest() const;
    std::shared_ptr<const CameraFrame> by_sequence(uint64_t sequence) const;
    std::size_t size() const;
    void clear();

private:
    mutable std::mutex mutex_;
    std::deque<std::shared_ptr<CameraFrame>> frames_;
    std::size_t capacity_ = 6;
};

PointSample query_point(const FrameBuffer &buffer, uint32_t u, uint32_t v,
                        uint64_t now_steady_ns, uint32_t max_age_ms,
                        bool require_trusted_time, uint64_t sequence = 0);

}  // namespace camera_adapter
