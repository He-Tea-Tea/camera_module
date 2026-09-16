// ROS2相机管理节点：把CameraRegistry封装成小脑PRIVATE/LOCAL_ONLY Topic、Service和Action。
#include "camera_manager/manager.hpp"

#include "camera_interfaces/action/capture_image.hpp"
#include "camera_interfaces/srv/get_point3_d.hpp"
#include "camera_interfaces/srv/query_camera_capability.hpp"
#include "camera_msgs/msg/camera_state.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace camera_manager {

namespace {

// CaptureImage未显式指定timeout时使用1秒默认值。
constexpr uint32_t kDefaultCaptureTimeoutMs = 1000;
// 防止错误请求让Action后台线程无边界等待；最大允许60秒。
constexpr uint32_t kMaximumCaptureTimeoutMs = 60000;

// 将纳秒时间戳转换成 ROS2 标准时间消息。
builtin_interfaces::msg::Time to_builtin_time(uint64_t nanoseconds) {
    builtin_interfaces::msg::Time result;
    result.sec = static_cast<int32_t>(nanoseconds / 1000000000ULL);
    result.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000ULL);
    return result;
}

// 仅允许本机绝对路径和 file:// URI，避免把证据误写到未知位置。
std::filesystem::path parse_local_path(const std::string &save_uri,
                                       const std::string &camera_name,
                                       uint64_t sequence) {
    std::string path = save_uri;
    constexpr const char *file_prefix = "file://";
    // file://URI转换成本地绝对路径。
    if (path.rfind(file_prefix, 0) == 0) {
        path.erase(0, std::char_traits<char>::length(file_prefix));
    } else if (path.find("://") != std::string::npos) {
        // v0.2.4仍禁止http/s3等远程URI，避免camera_module越权变成网络文件服务。
        throw camera_adapter::CameraError(
            camera_adapter::ErrorCode::InvalidRequest,
            "CaptureImage 当前只接受本机路径或 file:// URI");
    }
    // 空save_uri使用/tmp下基于相机名和sequence的默认PPM文件名。
    if (path.empty()) {
        path = "/tmp/" + camera_name + "_" + std::to_string(sequence) + ".ppm";
    }

    const std::filesystem::path result(path);
    // 相对路径依赖当前工作目录，容易造成证据位置歧义，因此强制绝对路径。
    if (!result.is_absolute()) {
        throw camera_adapter::CameraError(
            camera_adapter::ErrorCode::InvalidRequest,
            "CaptureImage 保存路径必须是绝对路径");
    }
    return result;
}

}  // namespace

// CameraManagerNode只负责ROS2控制面；真实硬件连接由CameraRegistry后台线程执行。
class CameraManagerNode final : public rclcpp::Node {
public:
    using GetPoint3D = camera_interfaces::srv::GetPoint3D;
    using QueryCapability = camera_interfaces::srv::QueryCameraCapability;
    using CaptureImage = camera_interfaces::action::CaptureImage;

    // 构造顺序：读取配置 -> 创建Registry -> 建立本地ROS2端点 -> 后台启动硬件。
    CameraManagerNode() : Node("camera_manager") {
        auto configs = read_parameters();
        registry_ = std::make_unique<CameraRegistry>(std::move(configs));
        registry_->initialize();

        // 必须先建立 ROS2 控制面，再让后台线程调用可能阻塞的厂商 SDK。
        create_local_endpoints();
        registry_->start();

        RCLCPP_INFO(get_logger(),
                    "相机管理器控制面已启动，硬件连接正在后台执行");
    }

    // 析构时先通知Action后台任务和Registry停止，再等待已经启动的抓拍future结束。
    ~CameraManagerNode() override {
        shutting_down_.store(true);
        if (registry_) {
            registry_->stop();
        }

        std::vector<std::future<void>> action_futures;
        {
            std::lock_guard<std::mutex> lock(action_mutex_);
            action_futures.swap(action_futures_);
        }
        for (auto &future : action_futures) {
            future.wait();
        }
    }

private:
    // 从ROS2参数读取多相机CameraConfig；所有字段最终仍会经过CameraConfig::validate()。
    std::vector<camera_adapter::CameraConfig> read_parameters() {
        // camera_names定义本次节点管理的逻辑相机集合。
        const auto names = declare_parameter<std::vector<std::string>>(
            "camera_names", {"head_camera"});
        std::vector<camera_adapter::CameraConfig> configs;

        for (const auto &name : names) {
            camera_adapter::CameraConfig config;
            config.name = name;
            const std::string prefix = name + ".";
            // 后端、设备型号和绑定信息。
            config.backend = declare_parameter<std::string>(prefix + "backend", "mock");
            config.model = declare_parameter<std::string>(prefix + "model", "Mock RGB-D");
            config.serial = declare_parameter<std::string>(prefix + "serial", "");
            config.ip_address = declare_parameter<std::string>(prefix + "ip_address", "");
            const int network_port = declare_parameter<int>(prefix + "network_port", 8090);
            if (network_port < 1 || network_port > 65535) {
                throw camera_adapter::CameraError(
                    camera_adapter::ErrorCode::InvalidConfig,
                    prefix + "network_port 必须为 1..65535");
            }
            config.network_port = static_cast<uint16_t>(network_port);

            // 坐标系、标定revision和V1.3稳定组件实例号。
            config.optical_frame = declare_parameter<std::string>(
                prefix + "optical_frame", name + "_color_optical_frame");
            config.calibration_revision = declare_parameter<std::string>(
                prefix + "calibration_revision", "factory-unverified");
            const int component_instance_id = declare_parameter<int>(
                prefix + "component_instance_id", static_cast<int>(configs.size() + 1));
            if (component_instance_id < 1 || component_instance_id > 65534) {
                throw camera_adapter::CameraError(
                    camera_adapter::ErrorCode::InvalidConfig,
                    prefix + "component_instance_id 必须为 1..65534");
            }
            config.component_instance_id = static_cast<uint16_t>(component_instance_id);

            // RGB/Depth目标码流尺寸和帧率。
            config.color_width = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "color_width", 640));
            config.color_height = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "color_height", 480));
            config.depth_width = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "depth_width", 640));
            config.depth_height = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "depth_height", 480));
            config.fps = static_cast<uint32_t>(declare_parameter<int>(prefix + "fps", 30));

            // SDK取帧、连续无帧、重连和本地缓存参数。
            config.wait_timeout_ms = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "wait_timeout_ms", 200));
            config.disconnect_timeout_ms = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "disconnect_timeout_ms", 2000));
            config.reconnect_delay_ms = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "reconnect_delay_ms", 1000));
            config.cache_capacity = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "cache_capacity", 6));

            // 本地观测新鲜度和RGB-D同步门控。
            config.max_frame_age_ms = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "max_frame_age_ms", 200));
            config.max_pair_delta_ms = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "max_pair_delta_ms", 35));

            // 可信采集时间策略；未实测时必须保持verified=false和bound=0。
            config.capture_delay_bound_verified = declare_parameter<bool>(
                prefix + "capture_delay_bound_verified", false);
            config.capture_delay_bound_ms = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "capture_delay_bound_ms", 0));

            // 生产设备绑定策略和统一有效深度范围。
            config.allow_unbound_device = declare_parameter<bool>(
                prefix + "allow_unbound_device", config.backend == "mock");
            config.min_depth_m = static_cast<float>(declare_parameter<double>(
                prefix + "min_depth_m", 0.2));
            config.max_depth_m = static_cast<float>(declare_parameter<double>(
                prefix + "max_depth_m", 6.0));

            // Mock专用确定性数据和故障注入参数。
            config.mock_depth_m = declare_parameter<double>(prefix + "mock_depth_m", 1.0);
            config.mock_disconnect_after_frames = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "mock_disconnect_after_frames", 0));
            config.mock_invalid_depth_every = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "mock_invalid_depth_every", 0));

            configs.push_back(std::move(config));
        }
        return configs;
    }

    // 为每台相机创建PRIVATE Service/Topic，并创建共享CaptureImage Action Server。
    void create_local_endpoints() {
        for (const auto &capability : registry_->capabilities()) {
            const auto name = capability.camera_name;

            // 像素三维查询Service；PUBLIC Router不得直接透传该端点。
            point_services_[name] = create_service<GetPoint3D>(
                "~/" + name + "/get_point_3d",
                [this](const std::shared_ptr<const GetPoint3D::Request> request,
                       std::shared_ptr<GetPoint3D::Response> response) {
                    handle_point(request, response);
                });

            // 单相机能力摘要Service；整机capability aggregator可以在小脑本地读取并转成公共强类型能力。
            capability_services_[name] = create_service<QueryCapability>(
                "~/" + name + "/query_capability",
                [this](const std::shared_ptr<const QueryCapability::Request> request,
                       std::shared_ptr<QueryCapability::Response> response) {
                    handle_capability(request, response);
                });

            // 状态Topic使用depth=1+transient_local，使新本地订阅者能立即看到最近状态。
            state_publishers_[name] = create_publisher<camera_msgs::msg::CameraState>(
                "~/" + name + "/state", rclcpp::QoS(1).transient_local());
        }

        // 抓拍是可等待、可取消、有反馈的本地长操作，因此使用Action而不是Service。
        capture_server_ = rclcpp_action::create_server<CaptureImage>(
            this, "~/capture_image",
            // Goal接纳阶段只做快速静态检查，不能阻塞在相机SDK上。
            [](const rclcpp_action::GoalUUID &,
               const std::shared_ptr<const CaptureImage::Goal> goal) {
                if (goal->camera_name.empty() || goal->timeout_ms > kMaximumCaptureTimeoutMs) {
                    return rclcpp_action::GoalResponse::REJECT;
                }
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            // 抓拍允许取消；执行线程会在等待和保存前检查cancel状态。
            [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<CaptureImage>>) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            // 每个Goal使用独立async任务，避免长I/O阻塞ROS2回调线程。
            [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<CaptureImage>> handle) {
                std::lock_guard<std::mutex> lock(action_mutex_);
                // 回收已经结束的future，并通过get传播线程内部未捕获异常。
                action_futures_.erase(
                    std::remove_if(
                        action_futures_.begin(), action_futures_.end(),
                        [](std::future<void> &future) {
                            if (future.wait_for(std::chrono::seconds(0)) !=
                                std::future_status::ready) {
                                return false;
                            }
                            future.get();
                            return true;
                        }),
                    action_futures_.end());
                action_futures_.emplace_back(std::async(
                    std::launch::async,
                    [this, handle]() { execute_capture(handle); }));
            });

        // 10Hz发布私有状态快照；状态不是高带宽图像数据。
        state_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                         [this]() { publish_states(); });
    }

    // 处理GetPoint3D：Registry负责帧选择和新鲜度，ROS层只做字段映射。
    void handle_point(const std::shared_ptr<const GetPoint3D::Request> request,
                      const std::shared_ptr<GetPoint3D::Response> response) {
        response->success = false;
        response->error_code = 0;

        try {
            const auto sample = registry_->query_point(request->camera_name, request->u,
                                                       request->v, request->require_trusted_time,
                                                       request->sequence);
            response->success = sample.code == camera_adapter::ErrorCode::Ok;
            response->x_m = sample.point_m[0];
            response->y_m = sample.point_m[1];
            response->z_m = sample.point_m[2];
            response->frame_sequence = sample.sequence;
            response->depth_valid = response->success;
            response->time_trusted = sample.frame && sample.frame->timing.trusted;
            response->optical_frame = sample.frame ? sample.frame->optical_frame : "";
            response->error_code = static_cast<uint32_t>(sample.code);
            response->message = sample.message;
            if (sample.frame) {
                // v0.2.4把session UUID和sequence一起返回，避免重连后sequence重复歧义。
                response->frame_session_uuid = sample.frame->provider_instance;
            }
            if (sample.frame && sample.frame->timing.capture_system_ns != 0) {
                response->capture_time = to_builtin_time(
                    sample.frame->timing.capture_system_ns);
            }
        } catch (const camera_adapter::CameraError &error) {
            response->error_code = static_cast<uint32_t>(error.code);
            response->message = error.what();
        }
    }

    // 处理单相机私有能力摘要；公共QueryCapabilities必须由整机层另行汇聚和强类型转换。
    void handle_capability(const std::shared_ptr<const QueryCapability::Request> request,
                           const std::shared_ptr<QueryCapability::Response> response) {
        response->success = false;
        response->error_code = 0;

        try {
            for (const auto &capability : registry_->capabilities()) {
                if (capability.camera_name != request->camera_name) {
                    continue;
                }
                response->success = true;
                response->component_instance_id = capability.component_instance_id;
                response->component_kind = capability.component_kind;
                response->device_id = capability.device_id;
                response->model = capability.model;
                response->serial = capability.serial;
                response->optical_frame = capability.optical_frame;
                response->calibration_revision = capability.calibration_revision;
                response->local_camera_available = capability.local_camera_available;
                response->capture_delay_bound_verified = capability.capture_delay_bound_verified;
                response->capture_delay_bound_ms = capability.capture_delay_bound_ms;
                response->max_frame_age_ms = capability.max_frame_age_ms;
                response->max_pair_delta_ms = capability.max_pair_delta_ms;
                response->hw_ptp_level = capability.hw_ptp_level;
                response->closed_loop_control_location = capability.closed_loop_control_location;
                response->message = "OK";
                return;
            }
            response->error_code = static_cast<uint32_t>(
                camera_adapter::ErrorCode::CameraNotFound);
            response->message = "未知相机: " + request->camera_name;
        } catch (const camera_adapter::CameraError &error) {
            response->error_code = static_cast<uint32_t>(error.code);
            response->message = error.what();
        }
    }

    // 发布每台相机的PRIVATE CameraState快照。
    void publish_states() {
        for (const auto &capability : registry_->capabilities()) {
            const auto status = registry_->status(capability.camera_name);
            camera_msgs::msg::CameraState message;
            message.camera_name = capability.camera_name;
            message.component_instance_id = capability.component_instance_id;
            message.component_kind = capability.component_kind;
            message.device_id = capability.device_id;
            message.state = static_cast<uint8_t>(status.state);
            message.rgb_ready = status.state == camera_adapter::CameraState::Streaming;
            message.depth_ready = status.state == camera_adapter::CameraState::Streaming;
            // 当前session至少缓存过一帧才声明已经验证D2C输出。
            message.depth_aligned_to_color = status.frame_count != 0;
            message.frame_session_uuid = status.frame_session_uuid;
            message.frame_sequence = status.last_sequence;
            if (status.last_capture_system_ns != 0) {
                message.last_frame_stamp = to_builtin_time(status.last_capture_system_ns);
            }
            message.time_trusted = status.time_trusted;
            message.pair_synchronized = status.pair_synchronized;
            message.optical_frame = capability.optical_frame;
            message.calibration_revision = capability.calibration_revision;
            message.last_error = status.last_error;
            message.error_code = static_cast<uint32_t>(status.last_error_code);
            state_publishers_.at(capability.camera_name)->publish(message);
        }
    }

    // 获取指定相机的新鲜度阈值；抓拍等待使用与三维查询一致的配置。
    uint32_t max_frame_age_ms(const std::string &camera_name) const {
        for (const auto &capability : registry_->capabilities()) {
            if (capability.camera_name == camera_name) {
                return capability.max_frame_age_ms;
            }
        }
        throw camera_adapter::CameraError(
            camera_adapter::ErrorCode::CameraNotFound,
            "未知相机: " + camera_name);
    }

    // 等待一帧满足Action的新鲜度、可信时间和取消/超时条件。
    std::shared_ptr<const camera_adapter::CameraFrame> wait_capture_frame(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CaptureImage>> &handle,
        camera_adapter::ErrorCode &error_code,
        std::string &message) {
        const auto goal = handle->get_goal();
        const uint32_t timeout_ms = goal->timeout_ms == 0
                                        ? kDefaultCaptureTimeoutMs
                                        : goal->timeout_ms;
        const uint32_t maximum_age_ms = max_frame_age_ms(goal->camera_name);
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(timeout_ms);

        while (!shutting_down_.load() && rclcpp::ok()) {
            // Action取消优先于继续等待帧。
            if (handle->is_canceling()) {
                error_code = camera_adapter::ErrorCode::Canceled;
                message = "抓拍请求已取消";
                return nullptr;
            }

            const auto frame = registry_->latest_frame(goal->camera_name);
            if (frame) {
                const uint64_t now_ns = camera_adapter::steady_now_ns();
                double age_ms = 0.0;
                if (frame->timing.trusted) {
                    // 有可信采集时间时使用保守采集年龄上界。
                    age_ms = camera_adapter::frame_age_upper_ms(frame->timing, now_ns);
                } else if (frame->timing.receive_steady_ns != 0 &&
                           now_ns >= frame->timing.receive_steady_ns) {
                    // 无可信采集时间时只使用本机接收年龄，不伪装成设备采集时间。
                    age_ms = static_cast<double>(
                        now_ns - frame->timing.receive_steady_ns) / 1e6;
                } else {
                    // 没有有效接收时间的帧不能被抓拍Action使用。
                    error_code = camera_adapter::ErrorCode::TimeUntrusted;
                    message = "最新帧没有可用接收时间";
                    age_ms = static_cast<double>(maximum_age_ms) + 1.0;
                }

                if (goal->require_trusted_time && !frame->timing.trusted) {
                    error_code = camera_adapter::ErrorCode::TimeUntrusted;
                    message = "最新帧没有可信采集时间";
                } else if (age_ms <= maximum_age_ms) {
                    return frame;
                } else if (error_code != camera_adapter::ErrorCode::TimeUntrusted) {
                    error_code = camera_adapter::ErrorCode::StaleFrame;
                    message = "最新帧已经过期";
                }
            } else {
                error_code = camera_adapter::ErrorCode::FrameNotFound;
                message = "没有可用帧";
            }

            // 达到Action deadline后返回最后一次明确错误原因。
            if (std::chrono::steady_clock::now() >= deadline) {
                return nullptr;
            }
            // 20ms轮询足以响应30FPS相机，同时不会忙等占满CPU。
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        error_code = camera_adapter::ErrorCode::Canceled;
        message = "相机管理器正在关闭";
        return nullptr;
    }

    // 执行一次CaptureImage Action：等待新鲜帧 -> 原子保存PPM -> 返回帧身份摘要。
    void execute_capture(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CaptureImage>> &handle) {
        auto result = std::make_shared<CaptureImage::Result>();
        const auto goal = handle->get_goal();
        result->camera_name = goal->camera_name;

        try {
            // 阶段1：等待满足要求的新鲜RGB-D证据帧。
            auto feedback = std::make_shared<CaptureImage::Feedback>();
            feedback->phase = CaptureImage::Feedback::PHASE_WAITING;
            feedback->message = "等待新鲜 RGB-D 帧";
            handle->publish_feedback(feedback);

            camera_adapter::ErrorCode wait_error = camera_adapter::ErrorCode::FrameNotFound;
            std::string wait_message = "没有可用帧";
            const auto frame = wait_capture_frame(handle, wait_error, wait_message);
            if (!frame) {
                result->success = false;
                result->error_code = static_cast<uint32_t>(wait_error);
                result->message = wait_message;
                if (wait_error == camera_adapter::ErrorCode::Canceled) {
                    handle->canceled(result);
                } else {
                    handle->abort(result);
                }
                return;
            }

            // 阶段2：锁定不可变shared_ptr帧，后续即使缓存滚动也不会改变证据内容。
            feedback->phase = CaptureImage::Feedback::PHASE_CAPTURING;
            feedback->message = "已锁定新鲜帧";
            handle->publish_feedback(feedback);

            if (handle->is_canceling() || shutting_down_.load()) {
                result->success = false;
                result->error_code = static_cast<uint32_t>(
                    camera_adapter::ErrorCode::Canceled);
                result->message = "抓拍请求已取消";
                handle->canceled(result);
                return;
            }

            // 阶段3：先写临时文件，再rename成最终文件，避免调用方看到半写入证据。
            feedback->phase = CaptureImage::Feedback::PHASE_SAVING;
            feedback->message = "正在原子保存 RGB 证据";
            handle->publish_feedback(feedback);

            const auto output_path = parse_local_path(
                goal->save_uri, goal->camera_name, frame->sequence);
            const auto temporary_path = output_path.string() +
                                        ".tmp." + std::to_string(frame->sequence);
            std::ofstream output(temporary_path, std::ios::binary | std::ios::trunc);
            if (!output) {
                throw camera_adapter::CameraError(
                    camera_adapter::ErrorCode::IoFailure,
                    "无法创建临时图像文件: " + temporary_path);
            }

            // PPM P6格式简单、无额外编码依赖，调试证据可被常见图像工具直接打开。
            output << "P6\n"
                   << frame->intrinsics.width << " "
                   << frame->intrinsics.height << "\n255\n";
            output.write(reinterpret_cast<const char *>(frame->rgb.data()),
                         static_cast<std::streamsize>(frame->rgb.size()));
            output.close();
            if (!output) {
                std::filesystem::remove(temporary_path);
                throw camera_adapter::CameraError(
                    camera_adapter::ErrorCode::IoFailure,
                    "图像临时文件写入失败");
            }
            std::filesystem::rename(temporary_path, output_path);

            // 成功结果返回session UUID+sequence，调用方可以唯一关联证据帧。
            result->success = true;
            result->image_uri = output_path.string();
            result->frame_session_uuid = frame->provider_instance;
            result->frame_sequence = frame->sequence;
            result->optical_frame = frame->optical_frame;
            result->time_trusted = frame->timing.trusted;
            result->error_code = 0;
            result->message = "OK";
            if (frame->timing.capture_system_ns != 0) {
                result->capture_time = to_builtin_time(
                    frame->timing.capture_system_ns);
            }

            feedback->phase = CaptureImage::Feedback::PHASE_DONE;
            feedback->message = "RGB 证据保存完成";
            handle->publish_feedback(feedback);
            handle->succeed(result);
        } catch (const camera_adapter::CameraError &error) {
            result->success = false;
            result->error_code = static_cast<uint32_t>(error.code);
            result->message = error.what();
            handle->abort(result);
        } catch (const std::exception &error) {
            result->success = false;
            result->error_code = static_cast<uint32_t>(
                camera_adapter::ErrorCode::IoFailure);
            result->message = error.what();
            handle->abort(result);
        }
    }

    std::unique_ptr<CameraRegistry> registry_;  // 核心多相机Registry。
    rclcpp::TimerBase::SharedPtr state_timer_;  // 10Hz私有状态发布定时器。
    rclcpp_action::Server<CaptureImage>::SharedPtr capture_server_;  // 本地抓拍Action Server。
    std::atomic<bool> shutting_down_{false};   // 节点析构/关闭标记。
    std::mutex action_mutex_;                   // 保护action_futures_容器。
    std::vector<std::future<void>> action_futures_;  // 后台抓拍任务集合。
    std::unordered_map<std::string, rclcpp::Service<GetPoint3D>::SharedPtr> point_services_;  // 每相机3D查询Service。
    std::unordered_map<std::string, rclcpp::Service<QueryCapability>::SharedPtr>
        capability_services_;                   // 每相机能力摘要Service。
    std::unordered_map<std::string, rclcpp::Publisher<camera_msgs::msg::CameraState>::SharedPtr>
        state_publishers_;                      // 每相机PRIVATE状态Publisher。
};

}  // namespace camera_manager

// 标准ROS2节点入口；Launch负责传入参数文件。
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<camera_manager::CameraManagerNode>());
    rclcpp::shutdown();
    return 0;
}
