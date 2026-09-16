#pragma once

// 有界帧缓存只保存已经脱离厂商 SDK 的自有内存帧。
#include "camera_adapter/camera_adapter.hpp"

#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>

namespace camera_adapter {

// 单像素三维查询结果；frame保留证据帧身份、session UUID、时间和标定revision。
struct PointSample {
    ErrorCode code = ErrorCode::NotReady;                    // 机器可判定结果码。
    std::string message;                                     // 人类可读诊断信息。
    std::shared_ptr<const CameraFrame> frame;                 // 实际用于查询的不可变帧；失败前若已定位到帧也会保留。
    uint64_t sequence = 0;                                   // 实际使用的当前session帧序号。
    std::array<double, 3> point_m{};                          // 光学坐标系三维点，单位米。
    double age_upper_ms = 0.0;                               // 查询时计算的帧年龄/接收年龄上界。
};

// 线程安全的有界帧缓存；容量小而固定，避免原始RGB-D无限增长占用内存。
class FrameBuffer {
public:
    explicit FrameBuffer(std::size_t capacity = 6);           // 创建指定容量的缓存。
    void set_capacity(std::size_t capacity);                  // 动态调整容量并立即裁剪最旧帧。
    void push(std::shared_ptr<CameraFrame> frame);             // 验证并压入一个完整统一帧。
    std::shared_ptr<const CameraFrame> latest() const;         // 返回最新帧，空缓存返回nullptr。
    std::shared_ptr<const CameraFrame> by_sequence(uint64_t sequence) const;  // 在当前缓存中按sequence反向查找。
    std::size_t size() const;                                 // 返回当前缓存帧数。
    void clear();                                             // 清空全部旧会话帧，重连时必须调用。

private:
    mutable std::mutex mutex_;                                // 保护frames_和capacity_。
    std::deque<std::shared_ptr<CameraFrame>> frames_;         // 按时间从旧到新保存帧。
    std::size_t capacity_ = 6;                                // 当前最大帧数。
};

// 查询指定或最新帧的像素三维坐标；sequence=0表示使用最新帧。
PointSample query_point(const FrameBuffer &buffer, uint32_t u, uint32_t v,
                        uint64_t now_steady_ns, uint32_t max_age_ms,
                        bool require_trusted_time, uint64_t sequence = 0);

}  // namespace camera_adapter
