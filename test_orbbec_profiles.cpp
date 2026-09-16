#include <iostream>
#include <memory>

#include <libobsensor/ObSensor.hpp>


// 打印指定传感器支持的全部视频码流配置。
void print_profiles(
    ob::Pipeline &pipeline,
    OBSensorType sensor_type,
    const std::string &sensor_name
) {
    std::cout << "\n========== " << sensor_name << " ==========" << std::endl;

    try {
        auto profiles = pipeline.getStreamProfileList(sensor_type);

        if (!profiles) {
            std::cout << "没有获得 Profile 列表" << std::endl;
            return;
        }

        const uint32_t count = profiles->getCount();

        std::cout << "Profile 数量: " << count << std::endl;

        for (uint32_t index = 0; index < count; ++index) {
            try {
                auto profile = profiles->getProfile(index);
                auto video_profile = profile->as<ob::VideoStreamProfile>();

                std::cout
                    << "[" << index << "] "
                    << video_profile->getWidth()
                    << "x"
                    << video_profile->getHeight()
                    << " @ "
                    << video_profile->getFps()
                    << " FPS"
                    << " format="
                    << static_cast<int>(video_profile->getFormat())
                    << std::endl;
            } catch (const std::exception &error) {
                std::cout
                    << "[" << index << "] 非视频 Profile: "
                    << error.what()
                    << std::endl;
            }
        }
    } catch (const std::exception &error) {
        std::cerr
            << sensor_name
            << " Profile 查询失败: "
            << error.what()
            << std::endl;
    }
}


int main() {
    try {
        std::cout
            << "[1] 创建 Orbbec SDK Context"
            << std::endl;

        ob::Context context;

        std::cout
            << "[2] 连接 Gemini335Le 192.168.1.10:8090"
            << std::endl;

        auto device = context.createNetDevice(
            "192.168.1.10",
            8090
        );

        if (!device) {
            std::cerr << "无法打开 Gemini335Le" << std::endl;
            return 1;
        }

        auto info = device->getDeviceInfo();

        std::cout
            << "设备: "
            << info->getName()
            << std::endl;

        std::cout
            << "序列号: "
            << info->getSerialNumber()
            << std::endl;

        // 使用已经打开的网络设备创建 Pipeline。
        ob::Pipeline pipeline(device);

        // 输出 Color 和 Depth 支持的全部 Profile。
        print_profiles(
            pipeline,
            OB_SENSOR_COLOR,
            "COLOR"
        );

        print_profiles(
            pipeline,
            OB_SENSOR_DEPTH,
            "DEPTH"
        );

        return 0;
    } catch (const std::exception &error) {
        std::cerr
            << "异常: "
            << error.what()
            << std::endl;

        return 1;
    }
}
