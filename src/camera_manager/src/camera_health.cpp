// 将内部状态转换为简短健康标签，便于日志和启动检查使用。
#include "camera_manager/manager.hpp"

namespace camera_manager {

const char *health_label(const CameraStatus &status) {
    switch (status.state) {
        case camera_adapter::CameraState::Streaming:
            return status.time_trusted && status.pair_synchronized ? "ready" : "degraded";
        case camera_adapter::CameraState::Connecting:
        case camera_adapter::CameraState::Reconnecting:
            return "starting";
        case camera_adapter::CameraState::Fault:
            return "fault";
        case camera_adapter::CameraState::Stopped:
        default:
            return "stopped";
    }
}

}  // namespace camera_manager
