# Imu节点

- 发布imu数据到话题{imu_data, imu_name, data}
- 感觉真没必要上多imu，大概率大多数时候instance字段都是imu0吧
- 将话题发布到{imu_control,  imu_name,  reset}来请求重置imu积分惯性导航
- 注意reset请求的类型是int
- 感觉整个imu裂成两半了，一半在硬件层，一半在tf层
- 发布imu速度：{imu_velocity, imu_name, data}