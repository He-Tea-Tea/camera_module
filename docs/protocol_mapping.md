# v0.2.4 与《具身本体大脑-小脑通信协议（ROS2）V1.3》映射

本仓库只实现相机小脑本地模块。V1.3的公司级公共ABI由整机 `robot_body_interfaces` 和 `robot_body_bootstrap_interfaces` 仓库维护，本仓库不能自行创建同名公共协议端点。

| V1.3要求 | camera_module v0.2.4 | 状态 |
| --- | --- | --- |
| 大脑不得直接访问厂商硬件 | Orbbec/RealSense SDK仅存在于`camera_driver` | 已实现 |
| 相机组件类别 | `component_kind=7` -> `COMPONENT_CAMERA` | 已实现 |
| 相机不占公共DeviceState | `device_id=255` -> `DEVICE_NONE` | 已实现 |
| component_instance_id稳定唯一 | YAML显式配置，范围1..65534 | 已实现 |
| PRIVATE只能LOCAL_ONLY | 4类相机IDL全部写入`camera_private_manifest` | 已实现草案 |
| 状态走Topic | `CameraState.msg`本地Topic | 已实现私有状态 |
| 等待/取消型操作走Action | `CaptureImage.action` | 已实现私有Action |
| RGB-D本地闭环 | FrameSync + Aggregate + D2C + FrameBuffer | 已实机验证 |
| 本地三维证据 | `GetPoint3D.srv` | 已实机验证 |
| 观测连续性 | `frame_session_uuid + frame_sequence` | v0.2.4新增 |
| 无PTP时最终闭环留小脑 | `hw_ptp=none`、`closed_loop=small_brain` | 已实现语义 |
| QueryCapabilities公共服务 | 由整机能力汇聚器转换本地相机能力 | 本仓库不实现公共服务 |
| Bootstrap/hash兼容握手 | 整机仓库实现 | 未在本仓库实现 |
| Router/SROS2跨域权限 | 当前仅有部署草案 | 待整机部署验证 |

## 私有接口

```text
CameraState.msg
GetPoint3D.srv
QueryCameraCapability.srv
CaptureImage.action
```

它们只允许：

```text
camera_manager
    ↕
小脑本地perception / skill / diagnostic
```

禁止：

```text
Brain -> camera_interfaces/*
Brain -> OrbbecSDK/librealsense
```

## QueryCameraCapability到公共QueryCapabilities

当前本地能力摘要可以为整机层提供：

```text
component_instance_id
component_kind=7
DEVICE_NONE语义
local_camera_available
hw_ptp_level
closed_loop_control_location
```

整机能力汇聚器负责转换为V1.3强类型：

```text
ComponentCapability.component_kind = COMPONENT_CAMERA
ComponentCapability.device_state_id = DEVICE_NONE
QueryCapabilities.local_camera_available = true/false
QueryCapabilities.hw_ptp_level = HW_PTP_NONE
QueryCapabilities.closed_loop_control_location = CLOSED_LOOP_CEREBELLUM
```

私有接口里保留的`"none"`和`"small_brain"`字符串不能直接复制进公共ABI，公共侧必须使用V1.3定义的uint8常量。

## 时间与新鲜度

真实设备默认：

```text
capture_delay_bound_verified=false
time_trusted=false
```

本地仍用接收侧steady clock做缓存年龄和超时判断，但不会把接收时间伪装为设备采集时间。只有实际测得可靠延迟上界后，才能开启可信采集时间。

## 发布说明

`release_contract/camera_private_manifest.yaml` 是本模块的机器可检查私有清单，**不是**V1.3的整机权威 `interface_manifest.yaml`。正式公共hash、RIHS01类型hash、Router成员、SROS2成员和release contract都应在整机发布仓库生成和冻结。
