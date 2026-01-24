# 用于调试整车观测器的detector模拟器

直接发“解算优化后“的位姿到rmviz，不涉及图像渲染的。

写游戏真好玩哈哈

目前的思路：

物理模拟用Box2D俯视角，按键和ui用ftxui，前端就由iceoryx输出到open3d写的rmviz可视化了

由外到里大概是app->robot->publisher，其中app到robot的线程安全由共享的锁保证，robot到publisher这层就用原子量（因为ftxui回调地址绑定没有原子指针的重载QAQ）
