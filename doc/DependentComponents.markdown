# 第三方依赖

- iceoryx：用于IPC
- openvino：用于推理
- gtsam：用于进行ba优化、滑动窗口优化、imu积分
- opencv：用于传统视觉
- eigen：用于坐标系变换
- yaml-cpp：用于配置文件
- reflect-cpp：用于加载配置文件
- open3d：用于可视化
- tinympc：用于云台控制
- fast_tf：用于坐标变换插值缓冲区
- cxxopts：用于解析cli输入

需要本地编译安装iceoryx、openvino、gtsam，下载open3d的预编译包

需要系统cmake版本大于3.23，需要libc++-dev包