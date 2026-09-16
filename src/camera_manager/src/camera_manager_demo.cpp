// 无 ROS2 环境下验证协议边界、本地查询路径和v0.2.4 session UUID语义的演示程序。
#include "camera_manager/manager.hpp"

#include <chrono>
#include <iostream>
#include <thread>

int main() {
    using namespace camera_adapter;
    using namespace camera_manager;

    // 构造一台小尺寸Mock相机，减少无硬件回归的CPU和内存开销。
    CameraConfig config;
    config.name = "head_camera";
    config.backend = "mock";
    config.model = "Mock RGB-D";
    config.color_width = 64;
    config.color_height = 48;
    config.depth_width = 64;
    config.depth_height = 48;
    config.fps = 30;
    config.cache_capacity = 4;
    config.max_frame_age_ms = 500;
    config.capture_delay_bound_verified = false;
    config.mock_depth_m = 1.2F;

    try {
        CameraRegistry registry({config});
        registry.initialize();

        // 第一次启动：等待几帧并验证三维查询。
        registry.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(120));
        const auto first_status = registry.status(config.name);
        const auto first_frame = registry.latest_frame(config.name);
        const auto sample = registry.query_point(config.name, 32, 24, true);
        if (!first_frame) {
            std::cerr << "camera_manager_demo: 第一次启动没有获得Mock帧\n";
            registry.stop();
            return 2;
        }
        const InstanceId first_session = first_frame->provider_instance;
        const uint64_t first_sequence = first_frame->sequence;

        // 停止后再次start，验证新streaming session UUID必须变化。
        registry.stop();
        registry.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(120));
        const auto second_frame = registry.latest_frame(config.name);
        if (!second_frame) {
            std::cerr << "camera_manager_demo: 第二次启动没有获得Mock帧\n";
            registry.stop();
            return 3;
        }
        const bool session_changed =
            !is_zero_instance(first_session) &&
            !is_zero_instance(second_frame->provider_instance) &&
            first_session != second_frame->provider_instance;

        // 输出稳定关键字，供camera-mock-smoke和CI脚本检查。
        std::cout << "state=" << health_label(first_status)
                  << " frames=" << first_status.frame_count
                  << " point_code=" << error_name(sample.code)
                  << " xyz=" << sample.point_m[0] << "," << sample.point_m[1]
                  << "," << sample.point_m[2]
                  << " first_sequence=" << first_sequence
                  << " session_changed=" << (session_changed ? "true" : "false")
                  << '\n';
        registry.stop();

        // Mock正常三维查询和session变化两项都通过才返回0。
        return sample.code == ErrorCode::Ok && session_changed ? 0 : 4;
    } catch (const CameraError &error) {
        std::cerr << "camera_manager_demo: " << error_name(error.code)
                  << ": " << error.what() << '\n';
        return 1;
    }
}
