// Orbbec Profile探针：只用于实验室确认真实设备支持的Color/Depth码流组合，不属于运行时节点。
#include <iostream>
#include <memory>
#include <string>

#include <libobsensor/ObSensor.hpp>

// 打印指定传感器支持的全部视频码流配置。
void print_profiles(ob::Pipeline &pipeline,
                    OBSensorType sensor_type,
                    const std::string &sensor_name) {
    // 打印传感器标题，便于区分COLOR和DEPTH。
    std::cout << "\n========== " << sensor_name << " ==========" << std::endl;
    try {
        // 从Pipeline查询指定传感器的全部StreamProfile。
        auto profiles = pipeline.getStreamProfileList(sensor_type);
        if (!profiles) {
            std::cout << "没有获得 Profile 列表" << std::endl;
            return;
        }
        // SDK返回的Profile数量。
        const uint32_t count = profiles->getCount();
        std::cout << "Profile 数量: " << count << std::endl;
        // 逐项转换为VideoStreamProfile并打印尺寸、帧率和格式枚举值。
        for (uint32_t index = 0; index < count; ++index) {
            try {
                auto profile = profiles->getProfile(index);
                auto video_profile = profile->as<ob::VideoStreamProfile>();
                std::cout << "[" << index << "] "
                          << video_profile->getWidth() << "x"
                          << video_profile->getHeight() << " @ "
                          << video_profile->getFps() << " FPS"
                          << " format=" << static_cast<int>(video_profile->getFormat())
                          << std::endl;
            } catch (const std::exception &error) {
                // 某些Profile不是视频流时只记录诊断，不中断其他Profile枚举。
                std::cout << "[" << index << "] 非视频 Profile: " << error.what() << std::endl;
            }
        }
    } catch (const std::exception &error) {
        std::cerr << sensor_name << " Profile 查询失败: " << error.what() << std::endl;
    }
}

int main(int argc, char **argv) {
    // 允许通过命令行覆盖设备IP和端口，默认保持当前Gemini335Le实验室配置。
    const std::string ip = argc >= 2 ? argv[1] : "192.168.1.10";
    const uint16_t port = argc >= 3 ? static_cast<uint16_t>(std::stoi(argv[2])) : 8090;
    try {
        // 创建Orbbec SDK Context。
        std::cout << "[1] 创建 Orbbec SDK Context" << std::endl;
        ob::Context context;
        // 直接连接指定网络设备，不进行全网段枚举。
        std::cout << "[2] 连接 Orbbec 网络设备 " << ip << ":" << port << std::endl;
        auto device = context.createNetDevice(ip.c_str(), port);
        if (!device) {
            std::cerr << "无法打开指定Orbbec设备" << std::endl;
            return 1;
        }
        // 打印设备身份，防止Profile结果来自错误设备。
        auto info = device->getDeviceInfo();
        std::cout << "设备: " << info->getName() << std::endl;
        std::cout << "序列号: " << info->getSerialNumber() << std::endl;
        // 使用已经打开的网络设备创建Pipeline。
        ob::Pipeline pipeline(device);
        // 输出Color和Depth支持的全部Profile。
        print_profiles(pipeline, OB_SENSOR_COLOR, "COLOR");
        print_profiles(pipeline, OB_SENSOR_DEPTH, "DEPTH");
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "异常: " << error.what() << std::endl;
        return 1;
    }
}
