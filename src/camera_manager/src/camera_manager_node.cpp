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

constexpr uint32_t kDefaultCaptureTimeoutMs = 1000;
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
    if (path.rfind(file_prefix, 0) == 0) {
        path.erase(0, std::char_traits<char>::length(file_prefix));
    } else if (path.find("://") != std::string::npos) {
        throw camera_adapter::CameraError(
            camera_adapter::ErrorCode::InvalidRequest,
            "CaptureImage 当前只接受本机路径或 file:// URI");
    }
    if (path.empty()) {
        path = "/tmp/" + camera_name + "_" + std::to_string(sequence) + ".ppm";
    }

    const std::filesystem::path result(path);
    if (!result.is_absolute()) {
        throw camera_adapter::CameraError(
            camera_adapter::ErrorCode::InvalidRequest,
            "CaptureImage 保存路径必须是绝对路径");
    }
    return result;
}

}  // namespace

class CameraManagerNode final : public rclcpp::Node {
public:
    using GetPoint3D = camera_interfaces::srv::GetPoint3D;
    using QueryCapability = camera_interfaces::srv::QueryCameraCapability;
    using CaptureImage = camera_interfaces::action::CaptureImage;

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
    std::vector<camera_adapter::CameraConfig> read_parameters() {
        const auto names = declare_parameter<std::vector<std::string>>(
            "camera_names", {"head_camera"});
        std::vector<camera_adapter::CameraConfig> configs;

        for (const auto &name : names) {
            camera_adapter::CameraConfig config;
            config.name = name;
            const std::string prefix = name + ".";
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
            config.color_width = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "color_width", 640));
            config.color_height = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "color_height", 480));
            config.depth_width = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "depth_width", 640));
            config.depth_height = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "depth_height", 480));
            config.fps = static_cast<uint32_t>(declare_parameter<int>(prefix + "fps", 30));
            config.wait_timeout_ms = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "wait_timeout_ms", 200));
            config.disconnect_timeout_ms = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "disconnect_timeout_ms", 2000));
            config.reconnect_delay_ms = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "reconnect_delay_ms", 1000));
            config.cache_capacity = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "cache_capacity", 6));
            config.max_frame_age_ms = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "max_frame_age_ms", 200));
            config.max_pair_delta_ms = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "max_pair_delta_ms", 35));
            config.capture_delay_bound_verified = declare_parameter<bool>(
                prefix + "capture_delay_bound_verified", false);
            config.capture_delay_bound_ms = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "capture_delay_bound_ms", 0));
            config.allow_unbound_device = declare_parameter<bool>(
                prefix + "allow_unbound_device", config.backend == "mock");
            config.min_depth_m = static_cast<float>(declare_parameter<double>(
                prefix + "min_depth_m", 0.2));
            config.max_depth_m = static_cast<float>(declare_parameter<double>(
                prefix + "max_depth_m", 6.0));
            config.mock_depth_m = declare_parameter<double>(prefix + "mock_depth_m", 1.0);
            config.mock_disconnect_after_frames = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "mock_disconnect_after_frames", 0));
            config.mock_invalid_depth_every = static_cast<uint32_t>(declare_parameter<int>(
                prefix + "mock_invalid_depth_every", 0));
            configs.push_back(std::move(config));
        }
        return configs;
    }

    void create_local_endpoints() {
        for (const auto &capability : registry_->capabilities()) {
            const auto name = capability.camera_name;
            point_services_[name] = create_service<GetPoint3D>(
                "~/" + name + "/get_point_3d",
                [this](const std::shared_ptr<const GetPoint3D::Request> request,
                       std::shared_ptr<GetPoint3D::Response> response) {
                    handle_point(request, response);
                });
            capability_services_[name] = create_service<QueryCapability>(
                "~/" + name + "/query_capability",
                [this](const std::shared_ptr<const QueryCapability::Request> request,
                       std::shared_ptr<QueryCapability::Response> response) {
                    handle_capability(request, response);
                });
            state_publishers_[name] = create_publisher<camera_msgs::msg::CameraState>(
                "~/" + name + "/state", rclcpp::QoS(1).transient_local());
        }

        capture_server_ = rclcpp_action::create_server<CaptureImage>(
            this, "~/capture_image",
            [](const rclcpp_action::GoalUUID &,
               const std::shared_ptr<const CaptureImage::Goal> goal) {
                if (goal->camera_name.empty() || goal->timeout_ms > kMaximumCaptureTimeoutMs) {
                    return rclcpp_action::GoalResponse::REJECT;
                }
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<CaptureImage>>) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<CaptureImage>> handle) {
                std::lock_guard<std::mutex> lock(action_mutex_);
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

        state_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                         [this]() { publish_states(); });
    }

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
            if (sample.frame && sample.frame->timing.capture_system_ns != 0) {
                response->capture_time = to_builtin_time(
                    sample.frame->timing.capture_system_ns);
            }
        } catch (const camera_adapter::CameraError &error) {
            response->error_code = static_cast<uint32_t>(error.code);
            response->message = error.what();
        }
    }

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
            message.depth_aligned_to_color = status.frame_count != 0;
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
                    age_ms = camera_adapter::frame_age_upper_ms(frame->timing, now_ns);
                } else if (frame->timing.receive_steady_ns != 0 &&
                           now_ns >= frame->timing.receive_steady_ns) {
                    age_ms = static_cast<double>(
                        now_ns - frame->timing.receive_steady_ns) / 1e6;
                }

                if (goal->require_trusted_time && !frame->timing.trusted) {
                    error_code = camera_adapter::ErrorCode::TimeUntrusted;
                    message = "最新帧没有可信采集时间";
                } else if (age_ms <= maximum_age_ms) {
                    return frame;
                } else {
                    error_code = camera_adapter::ErrorCode::StaleFrame;
                    message = "最新帧已经过期";
                }
            } else {
                error_code = camera_adapter::ErrorCode::FrameNotFound;
                message = "没有可用帧";
            }

            if (std::chrono::steady_clock::now() >= deadline) {
                return nullptr;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        error_code = camera_adapter::ErrorCode::Canceled;
        message = "相机管理器正在关闭";
        return nullptr;
    }

    void execute_capture(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CaptureImage>> &handle) {
        auto result = std::make_shared<CaptureImage::Result>();
        const auto goal = handle->get_goal();
        result->camera_name = goal->camera_name;

        try {
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

            result->success = true;
            result->image_uri = output_path.string();
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

    std::unique_ptr<CameraRegistry> registry_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp_action::Server<CaptureImage>::SharedPtr capture_server_;
    std::atomic<bool> shutting_down_{false};
    std::mutex action_mutex_;
    std::vector<std::future<void>> action_futures_;
    std::unordered_map<std::string, rclcpp::Service<GetPoint3D>::SharedPtr> point_services_;
    std::unordered_map<std::string, rclcpp::Service<QueryCapability>::SharedPtr>
        capability_services_;
    std::unordered_map<std::string, rclcpp::Publisher<camera_msgs::msg::CameraState>::SharedPtr>
        state_publishers_;
};

}  // namespace camera_manager

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<camera_manager::CameraManagerNode>());
    rclcpp::shutdown();
    return 0;
}
