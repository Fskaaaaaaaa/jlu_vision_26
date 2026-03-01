串口接收->发布：

- 敌人颜色
- GimbalInfo（包含角度角速度弹速）{gimbal_info, serial, data}
- 新收到的子弹Id

订阅aimcommand->串口发送：

- aimcommand

发布从里程计（odom）到云台（gimbal）的坐标系旋转变换。

需要配合static_tf_bc发布从云台到相机光学系的静态变换使用。