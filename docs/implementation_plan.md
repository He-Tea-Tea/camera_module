# 从 v0.1.1 到可联调版本的实施步骤

## 第一步：固定协议和设备身份

1. 复制 `release_contract` 到整机发布仓库，填写协议版本、ABI、公共接口源和最终 hash。
2. 为头部和腕部相机分配固定 `component_instance_id`，例如 1 和 2。
3. 给 Gemini335Le 固定 IP 或序列号；给 D436 填写真实序列号。不要让程序按发现顺序选择生产设备。
4. 填写 `optical_frame`、标定修订号和分辨率；先保持 RGB 与对齐深度输出同尺寸。

## 第二步：先用 Mock 打通本地数据面

1. 编译 `camera_adapter`、`camera_driver`、`camera_manager`。
2. 运行 `camera_manager_demo`，确认缓存、三维反投影、帧序号和时间可信标记工作。
3. 用故障注入参数验证断连、无效深度、过期帧、像素越界和未可信时间拒绝。
4. 只有 Mock 冒烟通过后才接真实 SDK，避免把硬件故障和接口错误混在一起。

## 第三步：接入 Orbbec Gemini335Le

1. 安装 Orbbec SDK v2 的头文件、共享库和 udev 规则。
2. 用 `-DCAMERA_DRIVER_ENABLE_ORBBEC=ON` 构建。
3. 先单独运行一台头部相机，确认 RGB、深度、帧同步、深度到彩色对齐和相机内参。
4. 记录设备时间戳与主机接收时间；没有测量到延迟上界时保持 `capture_delay_bound_verified=false`。
5. 断开网线或 USB，确认状态进入 `FAULT/RECONNECTING`，恢复后实例 ID 不变而帧序列继续从新会话开始。

## 第四步：接入 RealSense D436

1. 安装 librealsense、udev 规则和实际序列号设备。
2. 用 `-DCAMERA_DRIVER_ENABLE_REALSENSE=ON` 构建。
3. 确认 SDK 的 RGB8 顺序、深度单位、对齐输出尺寸和 timestamp domain。
4. 检查多台 RealSense 同时存在时仍按 serial 绑定。

## 第五步：接 ROS2 私有服务

1. 构建接口包，检查 `ros2 interface show camera_interfaces/srv/GetPoint3D` 与源码一致。
2. 使用本地服务查询像素三维点；要求可信时间时传 `require_trusted_time=true`。
3. 用 Action 获取本地图像证据 URI，不把 RGB/Depth 大块数据放进任务服务响应。
4. 将状态话题设置为可靠、瞬态本地；生产环境使用 SROS2 和白名单。

## 第六步：接入大脑而不泄漏硬件

1. 大脑只调用整机公共能力接口，例如视觉证据或目标确认能力。
2. Router 把公共请求路由到小脑；大脑不能发现或调用 `camera_interfaces`。
3. 小脑在执行抓取前重新确认对象身份、位姿、坐标变换、标定版本、新鲜度、可达性和碰撞安全。
4. 没有本地确认返回 `PERCEPTION_CONFIRMATION_REQUIRED`，丢失返回 `PERCEPTION_LOST`，过期返回 `STALE_OBJECT_POSE`。

## 第七步：发布前验收

- Mock、真实双相机、断连恢复、时间不可信、标定版本变化和私有域隔离均有日志证据。
- `interface_manifest.yaml` 的源码 hash、类型 hash、QoS、路由、版本和 ABI 已冻结。
- SAFE_IDLE、心跳丢失、安全停机和重新 Bootstrap 流程已由整机测试覆盖。
