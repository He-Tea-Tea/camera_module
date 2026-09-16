# Gemini335Le 实机 Profile 记录

本文件保留 v0.2.3 调试阶段对真实 `Orbbec Gemini 335Le` 的 Profile 枚举结论，并作为 v0.2.4 当前固定码流配置的依据。完整 Profile 可随时使用 `tools/orbbec_profile_probe.cpp` 在目标机重新枚举，因此源码包不再保存调试阶段生成的二进制可执行文件。

## 实测设备

```text
连接方式：Ethernet
IP：192.168.1.10
Port：8090
设备：Orbbec Gemini 335Le
Serial：CPEP163000LG
```

## 与当前配置直接相关的 Color Profile

实测列表中存在：

```text
640x400 @ 30 FPS format=22
```

Orbbec SDK 中当前实现使用：

```cpp
OB_FORMAT_RGB
```

因此 v0.2.4 固定为：

```text
Color = 640x400 @ 30 FPS / RGB
```

## 与当前配置直接相关的 Depth Profile

实测列表中 `640x400 @ 30 FPS` 存在多种深度格式，其中包括：

```text
640x400 @ 30 FPS format=24
640x400 @ 30 FPS format=12
640x400 @ 30 FPS format=8
```

当前 v0.2.4 延续已经跑通的：

```text
Depth = 640x400 @ 30 FPS / Y16
```

对应代码：

```cpp
pipeline_config_->enableVideoStream(
    OB_STREAM_DEPTH,
    config.depth_width,
    config.depth_height,
    config.fps,
    OB_FORMAT_Y16
);
```

不要改回早期使用的 `OB_FORMAT_Z16`；该组合曾导致：

```text
No matched profile found for:
{type: Depth, format: Z16, width: 640, height: 400, fps: 30}
```

## RGB-D FrameSet 规则

真实设备还验证了只启动 Color/Depth 不足以保证每个返回 FrameSet 都同时包含两类帧。因此当前必须保留：

```cpp
pipeline_config_->setFrameAggregateOutputMode(
    OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE
);
```

并在启动前：

```cpp
pipeline_->enableFrameSync();
```

同时使用：

```cpp
pipeline_config_->setAlignMode(ALIGN_D2C_SW_MODE);
```

最终数据契约为：

```text
Color 640x400 RGB
Depth 640x400 Y16
Frame Sync
All-type aggregate required
Software Depth-to-Color alignment
```

## 重新枚举方法

目标机已经安装 Orbbec SDK 后，可以编译：

```bash
cd ~/hxb_code/camera_module
source "$HOME/hxb_code/camera_sdk/activate_camera_sdks.sh"

g++ -std=c++17 \
  tools/orbbec_profile_probe.cpp \
  -I"$HOME/hxb_code/camera_sdk/install/orbbec/include" \
  -L"$HOME/hxb_code/camera_sdk/install/orbbec/lib" \
  -Wl,-rpath,"$HOME/hxb_code/camera_sdk/install/orbbec/lib" \
  -lOrbbecSDK \
  -o /tmp/orbbec_profile_probe

/tmp/orbbec_profile_probe 192.168.1.10 8090
```

如果未来更换相机固件、SDK版本、设备型号或连接方式，应重新枚举 Profile 后再修改生产配置，不应仅凭产品规格表猜测合法 Profile 组合。
