#include "camera_manager/manager.hpp"

#include "camera_interfaces/action/capture_image.hpp"
#include "camera_interfaces/srv/get_point3_d.hpp"
#include "camera_interfaces/srv/query_camera_capability.hpp"
#include "camera_msgs/msg/camera_state.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <algorithm>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace camera_manager {

namespace {

// 把纳秒时间戳转成 builtin_interfaces::msg::Time
// 避免依赖 rclcpp::Time::to_msg()，跨 ROS2 版本更稳
inline builtin_interfaces::msg::Time to_builtin_time(int64_t ns) {
    builtin_interfaces::msg::Time t;
    t.sec     = static_cast<int32_t>(ns / 1000000000LL);
    t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    return t;
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
        registry_->start();
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
            [](const rclcpp_action::GoalUUID &, std::shared_ptr<const CaptureImage::Goal>) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<CaptureImage>>) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<CaptureImage>> handle) {
                std::lock_guard<std::mutex> lock(action_mutex_);
                action_threads_.emplace_back([this, handle]() { execute_capture(handle); });
            });
        state_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                         [this]() { publish_states(); });
        RCLCPP_INFO(get_logger(), "相机管理器已启动，所有端点均为小脑本地接口");
    }

    ~CameraManagerNode() override {
        if (registry_) {
            registry_->stop();
        }
        std::lock_guard<std::mutex> lock(action_mutex_);
        for (auto &thread : action_threads_) {
            if (thread.joinable()) {
                thread.join();
            }
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
            config.optical_frame = declare_parameter<std::string>(
                prefix + "optical_frame", name + "_color_optical_frame");
            config.calibration_revision = declare_parameter<std::string>(
                prefix + "calibration_revision", "factory-unverified");
            config.component_instance_id = static_cast<uint16_t>(declare_parameter<int>(
                prefix + "component_instance_id", static_cast<int>(configs.size() + 1)));
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
                    static_cast<int64_t>(sample.frame->timing.capture_system_ns));
            }
        } catch (const camera_adapter::CameraError &error) {
            response->success = false;
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
                response->error_code = 0;
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
            response->message = "未知相机";
            // 若 camera_adapter::ErrorCode 有 UnknownCamera 就换成它
            // 这里先用 FrameNotFound 占位，你按实际枚举改
            response->error_code = static_cast<uint32_t>(
                camera_adapter::ErrorCode::FrameNotFound);
        } catch (const camera_adapter::CameraError &error) {
            response->success = false;
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
            message.rgb_ready = status.frame_count != 0;
            message.depth_ready = status.frame_count != 0;
            message.depth_aligned_to_color = true;
            message.frame_sequence = status.last_sequence;
            message.time_trusted = status.time_trusted;
            message.pair_synchronized = status.pair_synchronized;
            message.optical_frame = capability.optical_frame;
            message.calibration_revision = capability.calibration_revision;
            message.last_error = status.last_error;
            message.error_code = static_cast<uint32_t>(status.last_error_code);
            state_publishers_.at(capability.camera_name)->publish(message);
        }
    }

    void execute_capture(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CaptureImage>> &handle) {
        auto result = std::make_shared<CaptureImage::Result>();
        const auto goal = handle->get_goal();
        result->camera_name = goal->camera_name;

        try {
            auto feedback = std::make_shared<CaptureImage::Feedback>();
            feedback->phase = CaptureImage::Feedback::PHASE_WAITING;
            feedback->message = "等待最新帧";
            handle->publish_feedback(feedback);

            const auto frame = registry_->latest_frame(goal->camera_name);
            if (!frame || (goal->require_trusted_time && !frame->timing.trusted)) {
                result->success = false;
                result->error_code = static_cast<uint32_t>(
                    frame ? camera_adapter::ErrorCode::TimeUntrusted
                          : camera_adapter::ErrorCode::FrameNotFound);
                result->message = frame ? "最新帧没有可信时间" : "没有可用帧";
                handle->abort(result);
                return;
            }

            feedback->phase = CaptureImage::Feedback::PHASE_SAVING;
            feedback->message = "保存 RGB 证据";
            handle->publish_feedback(feedback);

            std::string path = goal->save_uri;
            if (path.empty()) {
                path = "/tmp/" + goal->camera_name + "_" +
                       std::to_string(frame->sequence) + ".ppm";
            }
            std::ofstream output(path, std::ios::binary);
            if (!output) {
                result->success = false;
                result->error_code = static_cast<uint32_t>(
                    camera_adapter::ErrorCode::IoFailure);
                result->message = "无法写入图像文件";
                handle->abort(result);
                return;
            }
            output << "P6\n"
                   << frame->intrinsics.width << " "
                   << frame->intrinsics.height << "\n255\n";
            output.write(reinterpret_cast<const char *>(frame->rgb.data()),
                         static_cast<std::streamsize>(frame->rgb.size()));

            result->success = output.good();
            result->image_uri = path;
            result->frame_sequence = frame->sequence;
            result->optical_frame = frame->optical_frame;
            result->time_trusted = frame->timing.trusted;
            result->error_code = result->success
                                     ? 0
                                     : static_cast<uint32_t>(
                                           camera_adapter::ErrorCode::IoFailure);
            result->message = result->success ? "OK" : "图像写入失败";
            if (frame->timing.capture_system_ns != 0) {
                result->capture_time = to_builtin_time(
                    static_cast<int64_t>(frame->timing.capture_system_ns));
            }

            if (result->success) {
                handle->succeed(result);
            } else {
                handle->abort(result);
            }
        } catch (const camera_adapter::CameraError &error) {
            result->success = false;
            result->error_code = static_cast<uint32_t>(error.code);
            result->message = error.what();
            handle->abort(result);
        } catch (const std::exception &e) {
            result->success = false;
            result->error_code = static_cast<uint32_t>(
                camera_adapter::ErrorCode::IoFailure);
            result->message = e.what();
            handle->abort(result);
        }
    }

    std::unique_ptr<CameraRegistry> registry_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp_action::Server<CaptureImage>::SharedPtr capture_server_;
    std::mutex action_mutex_;
    std::vector<std::thread> action_threads_;
    std::unordered_map<std::string, rclcpp::Service<GetPoint3D>::SharedPtr> point_services_;
    std::unordered_map<std::string, rclcpp::Service<QueryCapability>::SharedPtr> capability_services_;
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