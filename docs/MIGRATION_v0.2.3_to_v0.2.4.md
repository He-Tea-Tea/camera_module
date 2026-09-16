# 从 v0.2.3 升级到 v0.2.4

v0.2.4保留v0.2.3已经验证通过的Gemini335Le工作参数和本地端点名称，但私有IDL增加了会话UUID字段，因此升级后必须重新编译`camera_msgs -> camera_interfaces -> camera_manager`。

## 接口变化

新增：

```text
CameraState.frame_session_uuid
GetPoint3D.Response.frame_session_uuid
CaptureImage.Result.frame_session_uuid
```

现有调用方如果使用生成的ROS2类型正常重新编译，不需要改变请求字段。只有自己手工解析序列化布局的代码才需要同步更新。

## 清单变化

删除旧名称：

```text
release_contract/interface_manifest.yaml
```

改为：

```text
release_contract/camera_private_manifest.yaml
```

原因是该文件只描述camera_module私有接口，不是V1.3整机权威interface manifest。

## 数据变化

真实驱动现在会把低于`min_depth_m`或高于`max_depth_m`的深度直接标准化为0。因此升级前若上层曾依赖范围外原始深度值，需要改为通过配置放宽有效范围，而不是绕过模块过滤。

## 构建建议

升级后建议清理受IDL影响的包再编译：

```bash
cd ~/hxb_code/camera_module
rm -rf \
  build/camera_msgs build/camera_interfaces build/camera_manager \
  install/camera_msgs install/camera_interfaces install/camera_manager

./scripts/build_orbbec.sh
```

随后重新加载：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/hxb_code/camera_sdk/activate_camera_sdks.sh"
source "$HOME/hxb_code/camera_module/install/setup.bash"
```

再按README完成state、capability、GetPoint3D、invalid pixel、time trusted和CaptureImage验收。
