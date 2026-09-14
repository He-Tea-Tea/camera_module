# Camera Module

## 版本

**Version: v0.1.0**

**版本名称：相机模块初始框架**

---

# 1. 项目简介

Camera Module 是服务机器人“小脑（Cerebellum）”中的视觉设备管理模块。

当前阶段仅负责机器人 RGB-D 相机的软件架构设计与基础接口搭建，不包含机械臂、导航、任务调度等其他机器人能力。

本模块面向：

- Orbbec Gemini 335Le（环境感知相机）
- Intel RealSense D435i（腕部 Eye-in-Hand 相机）

设计目标：

> 将相机硬件差异隔离在小脑内部，使上层大脑只依赖统一视觉能力接口。

未来更换相机型号时，只需要增加或修改对应 Adapter 与 Driver，不影响任务系统、视觉算法以及大脑侧逻辑。

---

# 2. 系统定位

整体机器人架构：

```
                    Brain

                      |

          robot_body_interfaces

          （公共协议接口）

                      |

                 Cerebellum

                      |

              Camera Module

                      |

        -------------------------

        |                       |

   Gemini335Le              D435i

   Orbbec SDK            RealSense SDK

```


---

# 3. 设计原则

## 3.1 硬件隔离

上层模块禁止直接调用：

- Orbbec SDK
- RealSense SDK
- 相机底层驱动接口


错误：

```
Brain

 |

pyorbbecsdk

 |

Gemini335Le

```


正确：

```
Brain

 |

Camera Interface

 |

Camera Manager

 |

Camera Adapter

 |

Camera Driver

 |

Camera SDK

```

---

## 3.2 接口稳定

公共接口只描述：

- 相机能力
- 数据状态
- 三维查询
- 图像数据


不描述：

- 厂商型号
- SDK版本
- USB通信方式


---

## 3.3 模块化设计

相机模块划分：

```
camera_msgs

        ↓

camera_interfaces

        ↓

camera_manager

        ↓

camera_adapter

        ↓

camera_driver

        ↓

Camera SDK

```

---

# 4. 当前版本功能

## v0.1.0 已完成框架


### 已创建模块

```
camera_module/

├── camera_interfaces

├── camera_msgs

├── camera_manager

├── camera_adapter

├── camera_driver

├── camera_config

├── camera_tools

├── launch

├── docs

└── README.md

```


---

# 5. 模块说明


## camera_msgs

作用：

定义 ROS 消息。


包含：

```
CameraState.msg

CameraCapability.msg

CameraFrame.msg

DepthInfo.msg

```


用于描述：

- 相机状态
- 相机能力
- RGB-D数据状态


---

## camera_interfaces

作用：

定义大脑调用接口。


包含：

```
QueryCameraCapability.srv

GetPoint3D.srv

CaptureImage.action

```


提供：

- 能力查询
- 三维坐标查询
- 图像采集请求


---

## camera_manager

作用：

相机模块管理中心。


负责：

- 相机生命周期管理
- Adapter加载
- 状态发布
- 异常监控


---

## camera_adapter

作用：

相机统一抽象层。


定义：

所有相机必须实现的接口。


例如：

```
open()

close()

start()

stop()

getRGB()

getDepth()

getState()

```


该层禁止出现：

```
Orbbec SDK

RealSense SDK

```

---

## camera_driver

作用：

厂商SDK封装层。


例如：

```
camera_driver/

├── orbbec/

│   └── OrbbecSDK V2


└── realsense/

    └── librealsense

```


---

## camera_config

作用：

管理相机配置。


例如：

```
camera_config/

├── cameras.yaml

├── gemini335le.yaml

└── d435i.yaml

```


示例：

```yaml
camera:

  head:

    adapter: orbbec

    model: Gemini335Le


  wrist:

    adapter: realsense

    model: D435i

```

---

# 6. 当前版本目录

```
camera_module/

├── config/

├── docs/

├── launch/

├── src/

│
├── camera_adapter/

├── camera_driver/

├── camera_interfaces/

├── camera_manager/

├── camera_msgs/

└── camera_tools/


├── README.md

└── .gitignore

```

---

# 7. 开发路线

## v0.1.0

相机模块初始框架

完成：

- 项目结构
- ROS2 Package创建
- 消息接口设计
- 服务接口设计
- Adapter架构设计

---

# 8. 开发环境

目标环境：

```
Ubuntu 22.04

ROS2 Humble

C++17

Python3

OpenCV

Open3D

```


---

# 9. 编译方式

进入项目：

```bash
cd camera_module
```


编译：

```bash
colcon build
```


加载环境：

```bash
source install/setup.bash
```


---

# 10. 版本记录


## v0.1.0

日期：

2026-09


内容：

- 初始化项目结构
- 创建ROS2模块划分
- 建立相机接口设计
- 建立硬件隔离架构


---

# 11. 设计目标总结


Camera Module 不负责：

- 目标识别
- 任务规划
- 机械臂控制
- 导航


Camera Module 只负责：

```
设备接入

↓

数据获取

↓

状态管理

↓

统一接口输出

```


最终目标：

> 让机器人“大脑”只关心视觉能力，而不关心相机硬件实现。