 # 组件介绍

```
├── auto_aim           自瞄
├── auto_ballistic	   弹道闭环	
├── auto_buff		   开符
├── auto_engagement    火控组件
├── coord_transform    坐标系变换整合发布
├── hardware		   处理硬件通信
├── msgs			   iceoryx通信用payload
├── status_feedback	   可视化和语音报警程序
├── third_party		   一些第三方依赖
└── tools			   静态工具库
```

- 自瞄组件

> 由detector和tracker组成，detector检测装甲板并发布ba优化完的类型和位姿，tracker订阅并进行滑动窗口拟合敌方状态，发布锁定的target给auto_engagement来弹道计算。订阅图像、相机信息、坐标变换

- 弹道闭环组件

> 由detector和corrector组成，订阅auto_engagement发布的瞄准信息并发布云台偏置矫正量

- 开符组件

> 同自瞄组件

- 火控组件

> 订阅自瞄和打符组件发布的参数化运动学方程并根据其他字段还原variant，依次迭代解算瞄准角，向串口节点发布云台控制命令

- 坐标系组件

> 订阅硬件组件发布的imu数据并积分，依此在发布坐标系变换。同时也发布从参数读取的静态坐标系变换

- 可视化组件

> 包含基于open3d实现的类rviz可视化程序，订阅各功能组件的调试信息。订阅各个功能组件的heartbeat以实现离线报警