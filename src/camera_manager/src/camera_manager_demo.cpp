// 无 ROS2 环境下验证协议边界和本地查询路径的演示程序。
#include "camera_manager/manager.hpp"

#include <chrono>
#include <iostream>
#include <thread>

int main() {
    using namespace camera_adapter;
    using namespace camera_manager;
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
        registry.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(120));
        const auto status = registry.status(config.name);
        const auto sample = registry.query_point(config.name, 32, 24, true);
        std::cout << "state=" << health_label(status)
                  << " frames=" << status.frame_count
                  << " point_code=" << error_name(sample.code)
                  << " xyz=" << sample.point_m[0] << "," << sample.point_m[1]
                  << "," << sample.point_m[2] << '\n';
        registry.stop();
        return sample.code == ErrorCode::Ok ? 0 : 2;
    } catch (const CameraError &error) {
        std::cerr << "camera_manager_demo: " << error_name(error.code)
                  << ": " << error.what() << '\n';
        return 1;
    }
}
