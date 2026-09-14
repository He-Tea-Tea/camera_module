// 相机生命周期、采集线程和恢复策略实现。
#include "camera_manager/manager.hpp"

#include "camera_driver/mock_camera_driver.hpp"
#include "camera_driver/orbbec_camera_driver.hpp"
#include "camera_driver/realsense_camera_driver.hpp"

#include <chrono>
#include <thread>

namespace camera_manager {
using namespace camera_adapter;

CameraRegistry::CameraRegistry(std::vector<CameraConfig> configs) {
    for (auto &config : configs) {
        config.validate();
        const std::string name = config.name;
        if (records_.find(name) != records_.end()) {
            throw CameraError(ErrorCode::InvalidConfig, "相机名称重复: " + name);
        }
        records_.emplace(name, std::make_unique<Record>(std::move(config)));
    }
    if (records_.empty()) {
        throw CameraError(ErrorCode::InvalidConfig, "至少需要配置一台相机");
    }
}

CameraRegistry::~CameraRegistry() {
    stop();
}

std::unique_ptr<CameraAdapter> CameraRegistry::make_adapter(const CameraConfig &config) {
    if (config.backend == "mock") {
        return std::make_unique<camera_driver::MockCameraDriver>();
    }
    if (config.backend == "orbbec") {
        return std::make_unique<camera_driver::OrbbecCameraDriver>();
    }
    if (config.backend == "realsense") {
        return std::make_unique<camera_driver::RealSenseCameraDriver>();
    }
    throw CameraError(ErrorCode::UnsupportedBackend, "不支持的相机后端: " + config.backend);
}

void CameraRegistry::initialize() {
    std::lock_guard<std::mutex> registry_lock(mutex_);
    if (initialized_) {
        return;
    }
    try {
        for (auto &entry : records_) {
            auto &record = *entry.second;
            record.adapter = make_adapter(record.config);
            record.adapter->initialize(record.config);
            record.initialized = true;
            record.status.state = CameraState::Stopped;
        }
        initialized_ = true;
    } catch (...) {
        for (auto &entry : records_) {
            if (entry.second->adapter) {
                entry.second->adapter->stop();
            }
        }
        throw;
    }
}

void CameraRegistry::start() {
    std::unique_lock<std::mutex> registry_lock(mutex_);
    if (started_) {
        return;
    }
    if (!initialized_) {
        registry_lock.unlock();
        initialize();
        registry_lock.lock();
    }
    try {
        for (auto &entry : records_) {
            auto &record = *entry.second;
            record.adapter->start();
            record.stop_requested.store(false);
            record.started = true;
            {
                std::lock_guard<std::mutex> lock(record.mutex);
                record.status = CameraStatus{};
                record.status.state = CameraState::Connecting;
            }
            record.worker = std::thread(&CameraRegistry::worker_loop, this, std::ref(record));
        }
        started_ = true;
    } catch (...) {
        for (auto &entry : records_) {
            entry.second->stop_requested.store(true);
            entry.second->adapter->stop();
        }
        for (auto &entry : records_) {
            if (entry.second->worker.joinable()) {
                entry.second->worker.join();
            }
        }
        throw;
    }
}

void CameraRegistry::stop() noexcept {
    std::lock_guard<std::mutex> registry_lock(mutex_);
    if (!started_ && !initialized_) {
        return;
    }
    for (auto &entry : records_) {
        auto &record = *entry.second;
        record.stop_requested.store(true);
        if (record.adapter) {
            record.adapter->stop();
        }
    }
    for (auto &entry : records_) {
        auto &record = *entry.second;
        if (record.worker.joinable()) {
            record.worker.join();
        }
        record.started = false;
        std::lock_guard<std::mutex> lock(record.mutex);
        record.status.state = CameraState::Stopped;
    }
    started_ = false;
}

bool CameraRegistry::started() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return started_;
}

void CameraRegistry::set_error(Record &record, const CameraError &error) {
    std::lock_guard<std::mutex> lock(record.mutex);
    record.status.state = CameraState::Fault;
    record.status.last_error_code = error.code;
    record.status.last_error = error.what();
}

void CameraRegistry::worker_loop(Record &record) {
    while (!record.stop_requested.load()) {
        try {
            const auto frame = record.adapter->wait_frame(record.config.wait_timeout_ms);
            record.buffer.push(frame);
            std::lock_guard<std::mutex> lock(record.mutex);
            record.status.state = CameraState::Streaming;
            record.status.frame_count += 1;
            record.status.last_sequence = frame->sequence;
            record.status.last_receive_steady_ns = frame->timing.receive_steady_ns;
            record.status.time_trusted = frame->timing.trusted;
            record.status.pair_synchronized = frame->timing.pair_synchronized;
            record.status.last_error.clear();
            record.status.last_error_code = ErrorCode::Ok;
        } catch (const CameraError &error) {
            if (record.stop_requested.load()) {
                break;
            }
            set_error(record, error);
            if (error.code == ErrorCode::DeviceFailure || error.code == ErrorCode::Unsynchronized) {
                record.buffer.clear();
            }
            record.adapter->stop();
            std::this_thread::sleep_for(std::chrono::milliseconds(record.config.reconnect_delay_ms));
            if (record.stop_requested.load()) {
                break;
            }
            try {
                {
                    std::lock_guard<std::mutex> lock(record.mutex);
                    record.status.state = CameraState::Reconnecting;
                }
                record.adapter->start();
            } catch (const CameraError &restart_error) {
                set_error(record, restart_error);
            }
        } catch (const std::exception &error) {
            set_error(record, CameraError(ErrorCode::DeviceFailure, error.what()));
        }
    }
}

CameraRegistry::Record &CameraRegistry::get_record(const std::string &camera_name) const {
    const auto iterator = records_.find(camera_name);
    if (iterator == records_.end()) {
        throw CameraError(ErrorCode::CameraNotFound, "未知相机: " + camera_name);
    }
    return *iterator->second;
}

std::vector<CameraCapability> CameraRegistry::capabilities() const {
    std::lock_guard<std::mutex> registry_lock(mutex_);
    std::vector<CameraCapability> result;
    result.reserve(records_.size());
    for (const auto &entry : records_) {
        const auto &record = *entry.second;
        const auto description = record.adapter ? record.adapter->description() : DeviceDescription{};
        CameraCapability capability;
        capability.component_instance_id = record.config.component_instance_id;
        capability.camera_name = record.config.name;
        capability.model = description.model.empty() ? record.config.model : description.model;
        capability.serial = description.serial;
        capability.optical_frame = record.config.optical_frame;
        capability.calibration_revision = record.config.calibration_revision;
        capability.capture_delay_bound_verified = record.config.capture_delay_bound_verified;
        capability.capture_delay_bound_ms = record.config.capture_delay_bound_ms;
        capability.max_frame_age_ms = record.config.max_frame_age_ms;
        capability.max_pair_delta_ms = record.config.max_pair_delta_ms;
        result.push_back(std::move(capability));
    }
    return result;
}

CameraStatus CameraRegistry::status(const std::string &camera_name) const {
    const auto &record = get_record(camera_name);
    std::lock_guard<std::mutex> lock(record.mutex);
    return record.status;
}

std::shared_ptr<const CameraFrame> CameraRegistry::latest_frame(
    const std::string &camera_name) const {
    const auto &record = get_record(camera_name);
    return record.buffer.latest();
}

PointSample CameraRegistry::query_point(const std::string &camera_name, uint32_t u,
                                        uint32_t v, bool require_trusted_time,
                                        uint64_t sequence) const {
    const auto &record = get_record(camera_name);
    return camera_adapter::query_point(record.buffer, u, v, steady_now_ns(),
                                       record.config.max_frame_age_ms,
                                       require_trusted_time, sequence);
}

}  // namespace camera_manager
