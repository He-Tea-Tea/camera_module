# camera_module 实施计划

## 已完成：v0.2.4

当前版本完成协议边界收口和头部Gemini335Le本地数据面加固：

```text
设备固定绑定
RGB-D启动
FrameSync
完整FrameSet聚合
Depth->Color对齐
内参读取
米制深度
有效范围过滤
有界帧缓存
session UUID
三维反投影
能力摘要
抓拍Action
自动重连
时间可信度门控
Mock回归
V1.3私有清单校验
```

头部设备仍使用已实测：

```text
Gemini335Le
192.168.1.10:8090
CPEP163000LG
640x400@30
RGB + Y16
```

## 下一阶段：v0.3.0 双真实相机

接入 RealSense D436 前先保持v0.2.4数据契约不变。D436应固定真实serial并分配：

```text
head_camera  component_instance_id=1
wrist_camera component_instance_id=2
```

验收重点不是“能出图”，而是两台设备并发时：session、sequence、缓存、状态、标定revision、GetPoint3D和CaptureImage完全隔离；任一设备掉线不应导致另一台设备重启。

## 标定阶段

对每台相机生成受控标定制品，至少包含设备serial、分辨率、内参、D2C关系、手眼外参（如适用）、验证误差、工具版本、时间和SHA-256。正式部署配置中的`calibration_revision`必须引用实际制品，不能继续使用`factory-unverified`。

## 时间验证阶段

增加 timing benchmark，统计设备帧间隔、RGB/Depth pair delta、主机接收jitter、P50/P95/P99和最大值。在没有测量证据前保持：

```text
capture_delay_bound_verified=false
HW_PTP_NONE
CLOSED_LOOP_CEREBELLUM
```

## 整机公共协议阶段

公共能力不在camera_module中直接实现。整机仓库负责：

```text
GetProtocolManifest
正式interface_manifest.yaml
release_contract成员hash
QueryCapabilities
GetDetailedStatus
RobotState/SafetyState
WholeBody ExecuteTask
Router
SROS2
```

整机能力汇聚器读取camera_module的私有能力摘要，再生成`ComponentCapability/SkillCapability`。Brain不得发现`camera_interfaces`。

## 最终验收阶段

最终需要覆盖：双真实相机、断连恢复、错误序列号、非法Profile、无效深度、过期帧、session切换、时间不可信、标定revision变化、公共/私有域隔离、Bootstrap不兼容SAFE_IDLE以及上层ObjectRef本地重确认。
