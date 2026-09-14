# v0.2.0

## 已完成

- 建立统一 RGB-D 帧、内参、时间、错误码和设备描述契约。
- 增加 Orbbec Gemini335Le、RealSense D436 和确定性 Mock 驱动。
- 增加有界帧缓存、像素三维查询、过期检测和可信时间门控。
- 增加管理器生命周期、状态、断连重连和本地能力摘要。
- 修正 ROS2 action 的 Goal/Result/Feedback 顺序，补齐 rosidl 生成依赖。
- 增加私有状态消息、三维查询服务、能力查询服务和图像采集动作。
- 增加配置校验、Mock 冒烟脚本、协议 manifest、发布契约、QoS 和拓扑文件。

## 尚需整机仓库完成

- 将公共视觉能力接入 `robot_body_interfaces`。
- 由 Bootstrap 节点生成并比较 manifest、release contract 和 rosidl type hash。
- 在 Router/Bridge 和 SROS2 中落实私有端点隔离。
- 用真实标定和实测采集延迟替换配置模板中的占位信息。
