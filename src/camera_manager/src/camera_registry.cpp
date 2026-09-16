// 相机生命周期、后台连接、采集和自动恢复策略实现。
#include "camera_manager/manager.hpp"

#include "camera_driver/mock_camera_driver.hpp"
#include "camera_driver/orbbec_camera_driver.hpp"
#include "camera_driver/realsense_camera_driver.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>

namespace camera_manager {
using namespace camera_adapter;

namespace {

// stop最多等待厂商SDK线程1.5秒，之后采用shared_ptr保活+detach避免ROS2进程永久卡死。
constexpr auto kShutdownWait = std::chrono::milliseconds(1500);
// 可中断重连等待每50ms检查一次stop_requested。
constexpr auto kInterruptPoll = std::chrono::milliseconds(50);

// 统一后台线程日志格式，包含逻辑camera_name。
void log_phase(const CameraConfig &config, const std::string &message) {
    std::clog << "[camera_registry] [" << config.name << "] " << message << std::endl;
}

// 重置与当前streaming session强相关的状态，避免重连前旧帧状态泄漏到新会话。
void reset_session_status(CameraStatus &status) {
    status.frame_session_uuid = {};
    status.frame_count = 0;
    status.last_sequence = 0;
    status.last_receive_steady_ns = 0;
    status.last_capture_system_ns = 0;
    status.time_trusted = false;
    status.pair_synchronized = false;
}

}  // namespace

// 构造时校验每台相机配置和camera_name唯一性，但不连接任何真实设备。
CameraRegistry::CameraRegistry(std::vector<CameraConfig> configs) {
    for (auto &config : configs) {
        config.validate();
        const std::string name = config.name;
        if (records_.find(name) != records_.end()) {
            throw CameraError(ErrorCode::InvalidConfig, "相机名称重复: " + name);
        }
        records_.emplace(name, std::make_shared<Record>(std::move(config)));
    }
    if (records_.empty()) {
        throw CameraError(ErrorCode::InvalidConfig, "至少需要配置一台相机");
    }
}

// 析构统一走stop，确保后台线程尽量退出。
CameraRegistry::~CameraRegistry() {
    stop();
}

// 根据backend创建具体CameraAdapter；厂商类型只在camera_manager内部工厂可见。
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

// initialize只标记Registry可启动，避免厂商SDK阻塞调用线程/ROS2构造函数。
void CameraRegistry::initialize() {
    // v0.2.3 开始不在调用线程连接硬件，避免厂商 SDK 阻塞 ROS2 控制面。
    initialized_.store(true);
}

// 为每台相机创建独立后台线程；start可在stop后再次调用进行完整会话重建测试。
void CameraRegistry::start() {
    std::lock_guard<std::mutex> registry_lock(mutex_);
    if (started_.load()) {
        return;
    }
    if (!initialized_.load()) {
        initialize();
    }

    for (auto &entry : records_) {
        const auto &record = entry.second;
        if (record->worker.joinable()) {
            throw CameraError(ErrorCode::InvalidRequest,
                              "相机工作线程仍在运行: " + record->config.name);
        }
        record->stop_requested.store(false);
        record->worker_finished.store(false);
        record->buffer.clear();
        {
            std::lock_guard<std::mutex> lock(record->mutex);
            record->description = DeviceDescription{};
            record->status = CameraStatus{};
            record->status.state = CameraState::Connecting;
            record->status.state_since_steady_ns = steady_now_ns();
        }
        record->worker = std::thread(&CameraRegistry::worker_loop, record);
    }
    started_.store(true);
}

// 请求所有工作线程退出；厂商SDK永久阻塞时在有限等待后detach，保证ROS2进程可退出。
void CameraRegistry::stop() noexcept {
    std::vector<std::shared_ptr<Record>> records;
    {
        std::lock_guard<std::mutex> registry_lock(mutex_);
        if (!started_.exchange(false)) {
            return;
        }
        records.reserve(records_.size());
        for (const auto &entry : records_) {
            entry.second->stop_requested.store(true);
            records.push_back(entry.second);
        }
    }

    for (const auto &record : records) {
        if (!record->worker.joinable()) {
            continue;
        }
        std::unique_lock<std::mutex> finish_lock(record->finish_mutex);
        const bool finished = record->finish_condition.wait_for(
            finish_lock, kShutdownWait,
            [&record]() { return record->worker_finished.load(); });
        finish_lock.unlock();

        if (finished) {
            record->worker.join();
            continue;
        }

        // 厂商 SDK 可能在网络调用中永久阻塞。线程持有 Record 的 shared_ptr，
        // 因此分离后不会访问已释放内存，ROS2 进程也能立即退出。
        log_phase(record->config, "SDK 调用未在 1.5 秒内退出，分离阻塞线程");
        record->worker.detach();
    }
}

// 返回Registry当前是否已启动。
bool CameraRegistry::started() const {
    return started_.load();
}

// 把异常写入状态快照，同时清除“当前活动session”字段，避免Fault状态误带旧session身份。
void CameraRegistry::set_error(Record &record, const CameraError &error) {
    std::lock_guard<std::mutex> lock(record.mutex);
    record.status.state = CameraState::Fault;
    record.status.state_since_steady_ns = steady_now_ns();
    reset_session_status(record.status);
    record.status.last_error_code = error.code;
    record.status.last_error = error.what();
}

// 重连等待拆成小片段，stop请求可以快速打断长reconnect_delay。
bool CameraRegistry::wait_interruptible(Record &record, uint32_t delay_ms) {
    uint32_t waited_ms = 0;
    while (!record.stop_requested.load() && waited_ms < delay_ms) {
        const uint32_t slice_ms = std::min<uint32_t>(
            delay_ms - waited_ms, static_cast<uint32_t>(kInterruptPoll.count()));
        std::this_thread::sleep_for(std::chrono::milliseconds(slice_ms));
        waited_ms += slice_ms;
    }
    return !record.stop_requested.load();
}

// 单台相机后台主循环：连接 -> start -> 持续取帧 -> 故障 -> stop -> 延时 -> 重连。
void CameraRegistry::worker_loop(const std::shared_ptr<Record> &record) {
    bool first_attempt = true;

    while (!record->stop_requested.load()) {
        std::unique_ptr<CameraAdapter> adapter;
        try {
            {
                std::lock_guard<std::mutex> lock(record->mutex);
                record->status.state = first_attempt
                                           ? CameraState::Connecting
                                           : CameraState::Reconnecting;
                record->status.state_since_steady_ns = steady_now_ns();
                // 每次连接尝试先清掉旧session状态；真正第一帧到达后再写入新UUID和sequence。
                reset_session_status(record->status);
            }

            log_phase(record->config, first_attempt ? "开始初始化设备" : "开始重新连接设备");
            adapter = make_adapter(record->config);
            adapter->initialize(record->config);
            if (record->stop_requested.load()) {
                break;
            }

            log_phase(record->config, "设备初始化完成，开始启动 RGB-D 数据流");
            adapter->start();
            const auto description = adapter->description();
            {
                std::lock_guard<std::mutex> lock(record->mutex);
                record->description = description;
                // start成功但第一组合法RGB-D尚未进入缓存前仍保持Connecting。
                record->status.state = CameraState::Connecting;
                record->status.state_since_steady_ns = steady_now_ns();
                record->status.last_error.clear();
                record->status.last_error_code = ErrorCode::Ok;
            }
            log_phase(record->config, "RGB-D 数据流已启动，等待第一组同步帧");

            // last_frame_ns用于判断连续无帧时间，而不是单次wait timeout。
            uint64_t last_frame_ns = steady_now_ns();
            while (!record->stop_requested.load()) {
                try {
                    const auto frame = adapter->wait_frame(record->config.wait_timeout_ms);
                    // FrameBuffer::push会统一验证session、sequence、内参、数据长度和RGB-D同步。
                    record->buffer.push(frame);
                    last_frame_ns = steady_now_ns();
                    {
                        std::lock_guard<std::mutex> lock(record->mutex);
                        record->status.state = CameraState::Streaming;
                        record->status.state_since_steady_ns = steady_now_ns();
                        record->status.frame_session_uuid = frame->provider_instance;
                        record->status.frame_count += 1;
                        record->status.last_sequence = frame->sequence;
                        record->status.last_receive_steady_ns = frame->timing.receive_steady_ns;
                        record->status.last_capture_system_ns = frame->timing.capture_system_ns;
                        record->status.time_trusted = frame->timing.trusted;
                        record->status.pair_synchronized = frame->timing.pair_synchronized;
                        record->status.last_error.clear();
                        record->status.last_error_code = ErrorCode::Ok;
                    }
                } catch (const CameraError &error) {
                    // 单次Timeout不立即重连；只有连续无帧超过disconnect_timeout才判会话失败。
                    if (error.code != ErrorCode::Timeout) {
                        throw;
                    }
                    const uint64_t elapsed_ms = (steady_now_ns() - last_frame_ns) / 1000000ULL;
                    if (elapsed_ms >= record->config.disconnect_timeout_ms) {
                        throw CameraError(
                            ErrorCode::Timeout,
                            "连续 " + std::to_string(elapsed_ms) + " ms 未收到 RGB-D 帧");
                    }
                }
            }
        } catch (const CameraError &error) {
            if (!record->stop_requested.load()) {
                set_error(*record, error);
                log_phase(record->config,
                          std::string("设备故障: ") + error_name(error.code) + ": " + error.what());
            }
        } catch (const std::exception &error) {
            if (!record->stop_requested.load()) {
                const CameraError wrapped(ErrorCode::DeviceFailure, error.what());
                set_error(*record, wrapped);
                log_phase(record->config, std::string("未分类设备故障: ") + error.what());
            }
        }

        // 每次连接尝试结束都先尽力停止当前adapter。
        if (adapter) {
            adapter->stop();
        }
        // 旧session帧绝不能跨重连保留。
        record->buffer.clear();
        if (record->stop_requested.load()) {
            break;
        }

        first_attempt = false;
        if (!wait_interruptible(*record, record->config.reconnect_delay_ms)) {
            break;
        }
    }

    // 工作线程退出后状态固定为Stopped，并清除活动session字段。
    {
        std::lock_guard<std::mutex> lock(record->mutex);
        record->status.state = CameraState::Stopped;
        record->status.state_since_steady_ns = steady_now_ns();
        reset_session_status(record->status);
    }
    record->worker_finished.store(true);
    record->finish_condition.notify_all();
    log_phase(record->config, "相机工作线程已退出");
}

// 按逻辑名查找Record；不存在时抛结构化CameraNotFound。
CameraRegistry::Record &CameraRegistry::get_record(const std::string &camera_name) const {
    const auto iterator = records_.find(camera_name);
    if (iterator == records_.end()) {
        throw CameraError(ErrorCode::CameraNotFound, "未知相机: " + camera_name);
    }
    return *iterator->second;
}

// 汇总所有相机当前能力；这是PRIVATE数据源，不直接等于V1.3公共QueryCapabilities。
std::vector<CameraCapability> CameraRegistry::capabilities() const {
    std::lock_guard<std::mutex> registry_lock(mutex_);
    std::vector<CameraCapability> result;
    result.reserve(records_.size());
    for (const auto &entry : records_) {
        const auto &record = *entry.second;
        std::lock_guard<std::mutex> record_lock(record.mutex);
        CameraCapability capability;
        capability.component_instance_id = record.config.component_instance_id;
        capability.camera_name = record.config.name;
        capability.model = record.description.model.empty()
                               ? record.config.model
                               : record.description.model;
        capability.serial = record.description.serial.empty()
                                ? record.config.serial
                                : record.description.serial;
        capability.optical_frame = record.config.optical_frame;
        capability.calibration_revision = record.config.calibration_revision;
        capability.local_camera_available = record.status.state == CameraState::Streaming;
        capability.capture_delay_bound_verified = record.config.capture_delay_bound_verified;
        capability.capture_delay_bound_ms = record.config.capture_delay_bound_ms;
        capability.max_frame_age_ms = record.config.max_frame_age_ms;
        capability.max_pair_delta_ms = record.config.max_pair_delta_ms;
        result.push_back(std::move(capability));
    }
    return result;
}

// 返回指定相机状态快照；连接阶段长时间无进展时在读取侧给出Fault诊断视图。
CameraStatus CameraRegistry::status(const std::string &camera_name) const {
    const auto &record = get_record(camera_name);
    std::lock_guard<std::mutex> lock(record.mutex);
    CameraStatus result = record.status;
    if ((result.state == CameraState::Connecting ||
         result.state == CameraState::Reconnecting) &&
        result.state_since_steady_ns != 0) {
        const uint64_t elapsed_ms =
            (steady_now_ns() - result.state_since_steady_ns) / 1000000ULL;
        if (elapsed_ms >= record.config.disconnect_timeout_ms) {
            result.state = CameraState::Fault;
            result.last_error_code = ErrorCode::Timeout;
            result.last_error = "SDK连接超过 " + std::to_string(elapsed_ms) + " ms";
        }
    }
    return result;
}

// 获取指定相机当前缓存最新帧。
std::shared_ptr<const CameraFrame> CameraRegistry::latest_frame(
    const std::string &camera_name) const {
    const auto &record = get_record(camera_name);
    return record.buffer.latest();
}

// 调用camera_adapter统一查询逻辑，Registry只补充每台相机自己的max_frame_age_ms。
PointSample CameraRegistry::query_point(const std::string &camera_name, uint32_t u,
                                        uint32_t v, bool require_trusted_time,
                                        uint64_t sequence) const {
    const auto &record = get_record(camera_name);
    return camera_adapter::query_point(record.buffer, u, v, steady_now_ns(),
                                       record.config.max_frame_age_ms,
                                       require_trusted_time, sequence);
}

}  // namespace camera_manager
