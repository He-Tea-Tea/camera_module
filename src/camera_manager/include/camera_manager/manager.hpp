#pragma once

// 相机管理器只编排生命周期和本地缓存，不把厂商 SDK 暴露给上层。
#include "camera_adapter/camera_adapter.hpp"
#include "camera_adapter/frame_buffer.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace camera_manager {

struct CameraCapability {
    uint16_t component_instance_id = 0;
    uint8_t component_kind = 7;
    uint8_t device_id = 255;
    std::string camera_name;
    std::string model;
    std::string serial;
    std::string optical_frame;
    std::string calibration_revision;
    bool local_camera_available = true;
    bool capture_delay_bound_verified = false;
    uint32_t capture_delay_bound_ms = 0;
    uint32_t max_frame_age_ms = 0;
    uint32_t max_pair_delta_ms = 0;
    std::string closed_loop_control_location = "small_brain";
    std::string hw_ptp_level = "none";
};

struct CameraStatus {
    camera_adapter::CameraState state = camera_adapter::CameraState::Stopped;
    uint64_t frame_count = 0;
    uint64_t last_sequence = 0;
    uint64_t last_receive_steady_ns = 0;
    bool time_trusted = false;
    bool pair_synchronized = false;
    std::string last_error;
    camera_adapter::ErrorCode last_error_code = camera_adapter::ErrorCode::Ok;
};

const char *health_label(const CameraStatus &status);

class CameraRegistry {
public:
    explicit CameraRegistry(std::vector<camera_adapter::CameraConfig> configs);
    ~CameraRegistry();

    void initialize();
    void start();
    void stop() noexcept;
    bool started() const;

    std::vector<CameraCapability> capabilities() const;
    CameraStatus status(const std::string &camera_name) const;
    std::shared_ptr<const camera_adapter::CameraFrame> latest_frame(
        const std::string &camera_name) const;
    camera_adapter::PointSample query_point(
        const std::string &camera_name, uint32_t u, uint32_t v,
        bool require_trusted_time, uint64_t sequence = 0) const;

private:
    struct Record {
        camera_adapter::CameraConfig config;
        std::unique_ptr<camera_adapter::CameraAdapter> adapter;
        camera_adapter::FrameBuffer buffer;
        mutable std::mutex mutex;
        std::thread worker;
        std::atomic<bool> stop_requested{false};
        bool initialized = false;
        bool started = false;
        CameraStatus status;

        explicit Record(camera_adapter::CameraConfig value)
            : config(std::move(value)), buffer(config.cache_capacity) {}
    };

    static std::unique_ptr<camera_adapter::CameraAdapter> make_adapter(
        const camera_adapter::CameraConfig &config);
    static void set_error(Record &record, const camera_adapter::CameraError &error);
    void worker_loop(Record &record);
    Record &get_record(const std::string &camera_name) const;

    std::unordered_map<std::string, std::unique_ptr<Record>> records_;
    mutable std::mutex mutex_;
    bool initialized_ = false;
    bool started_ = false;
};

}  // namespace camera_manager
