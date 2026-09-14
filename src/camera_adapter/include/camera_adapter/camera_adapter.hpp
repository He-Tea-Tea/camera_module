// 相机抽象接口
// 定义所有相机驱动必须实现的统一接口。
// 上层模块只依赖该接口，不直接依赖具体厂商SDK。
// 例如：Orbbec SDK、RealSense SDK 都需要通过该接口接入。

class CameraAdapter
{

public:

    // 初始化相机设备
    // 负责加载设备参数、创建SDK实例、完成硬件连接等。
    // 返回true表示初始化成功。
    virtual bool initialize() = 0;


    // 启动相机数据流
    // 开启RGB、Depth等数据采集线程。
    // 返回true表示启动成功。
    virtual bool start() = 0;


    // 停止相机数据流
    // 释放采集线程，关闭数据输出。
    // 返回true表示停止成功。
    virtual bool stop() = 0;


    // 获取RGB图像数据
    // 不关心底层相机型号，只提供统一图像接口。
    // 具体实现由不同相机Adapter完成。
    virtual bool getRGB() = 0;


    // 获取Depth深度数据
    // 用于三维定位、距离测量等功能。
    // 返回true表示获取有效深度数据。
    virtual bool getDepth() = 0;


    // 获取相机当前状态
    // 用于反馈设备连接状态、异常信息等。
    virtual bool getState() = 0;


};