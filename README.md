# jlu_vision_26

一个基于iceoryx零拷贝通信和gtsam重投影因子图优化的robomaster自瞄框架。

## 目录

- [特色与亮点](#特色与亮点)

- [文件目录说明](#文件目录说明)
- [开发的架构](#开发的架构)
- [部署](#部署)
- [作者](#作者)
- [鸣谢](#鸣谢)

### 特色与亮点

在不依赖 ROS 的前提下，实现了一套低延迟、易维护的视觉链路。

同时，结合了重投影误差和因子图优化，实现了高性能鲁棒的整车状态观测器，为反陀螺/能量机关提供了高质量的数据保障。

本项目无ai直接生成的代码，所有代码均为人类编写或经过人类重构，agent仅在调试阶段引入。

### 文件目录说明
```
filetree 
├── assets  运行时依赖
├── configs  配置文件
├── src
│   ├── auto_aim 自瞄组件
│   ├── auto_buff 能量机关组件
│   ├── common_defs 通用定义
│   ├── hardware 硬件io组件
│   ├── odom_coord 坐标系管理组件
│   ├── status_feedback 状态反馈组件
│   ├── third_party 第三方依赖
│   └── tools 工具代码
├── LICENSE.txt
├── README.md
├── startup 自启动脚本
└── xmake.lua
```

### 开发的架构

采用类君瞄的架构，但并未引入ros依赖。

使用共享内存进行跨进程通信，[fast_tf](https://github.com/dorezyuk/fast_tf)管理坐标系变换。

### 部署

- 这个自瞄的第三方依赖确实有点太多了，让配环境变成了一个极其折磨的事情

建议使用ubuntu22

源码编译安装iceoryx和gtsam（注意需要是4.3版本的）

安装openvino2024和GPU驱动（建议参考同济25赛季开源的README）

安装libc++的abi（为了链接open3d）

安装ceres

建议升级下系统cmake版本

安装xmake进行编译，双手离开键盘，祈祷xmake能自动处理安装好剩下的依赖

- xmake解决依赖基本都是源码安装，建议opencv之类的巨无霸库就拿apt装吧

安装海康和大恒的SDK

最后安装下process-compose，用来管理节点的启动

### 作者

冯洪叡 、张育宁

### 版权说明

该项目签署了MIT 授权许可，详情请参阅 LICENSE.txt

### 鸣谢


- [同济大学 sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25)
- [中南大学 FYT2024_vision](https://github.com/CSU-FYT-Vision/FYT2024_vision)
- [深圳大学 RobotDetectionModel](https://github.com/broalantaps/RobotDetectionModel)
- [君佬 rm_vision](https://github.com/chenjunnn/rm_vision)
- [吉林大学 JLU_VISION](https://github.com/liuy616161/JLU_VISION)
