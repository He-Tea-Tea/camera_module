# 协议映射和边界

本目录实现的是相机小脑模块。上传的 V1.3 协议要求 `interface_manifest.yaml` 与真实 rosidl 源码同时作为权威；因此本仓库只登记相机私有端点，整机公共接口仍需在 `robot_body_interfaces` 仓库中实现。

| 协议要求 | 本实现 | 验收方式 |
| --- | --- | --- |
| 大脑不直接访问硬件 | Orbbec/RealSense 只在 `camera_driver` 中出现 | 检查 `camera_manager` 和脑端依赖 |
| 相机属于 `COMPONENT_CAMERA=7` | `CameraCapability.component_kind=7` | 调用能力查询 |
| 无 DeviceState 的相机使用 `DEVICE_NONE=255` | 能力摘要默认 `device_id=255` | 检查 manifest |
| 组件实例号固定 | YAML 中显式填写 `component_instance_id` | 配置校验工具 |
| RGB-D 成对且深度对齐 | `CameraFrame::validate()` 强制检查 | Mock 冒烟测试 |
| 旧帧不能伪装新鲜 | 不使用接收时间刷新设备时间；查询有 `max_frame_age_ms` 和可信时间门 | 过期/不可信测试 |
| 没有硬件 PTP 时闭环留在小脑 | `closed_loop_control_location=small_brain` | 能力摘要和发布契约 |
| 原始图像不进任务热路径 | Action 返回本地文件证据 URI；私有原始数据面 | 检查路由白名单 |
| 私有接口不可被大脑调用 | manifest 标为 `PRIVATE/LOCAL_ONLY`，launch 不创建公共桥接 | SROS2/发现服务器配置 |
| 启动先握手再查能力 | 发布前需由整机 Bootstrap 节点比较版本和 hash | 联调启动顺序 |

## 需要整机仓库继续完成的部分

1. 在 `robot_body_bootstrap_interfaces` 中落地协议版本、ABI、manifest 和 type hash 比较。
2. 在 `robot_body_interfaces` 中把相机能力映射成公共语义能力，例如“获取视觉证据”，不要直接暴露 `GetPoint3D`。
3. 在 Router/Bridge 上配置静态 peer、SROS2、服务白名单和私有域隔离。
4. 将本模块的 `QueryCameraCapability` 结果转换为整机 `QueryCapabilities.srv` 的 `ComponentCapability` 和 `SkillCapability`。
5. 对目标物体执行本地重确认、坐标变换、标定版本和观测新鲜度检查；本模块只提供相机证据。
