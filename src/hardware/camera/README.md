# 相机节点

## 读取参数：

- 相机内外参
- 默认曝光增益
- 相机类型（大恒/海康/视频）

## 发布：

- 捕获的图像{image_raw, camera_name, data}
- 相机内参外参 {camera_info, camera_name, data}
- instance字段区分不同相机来源，为以后可能的多相机做准备

## 订阅话题：

- 更改增益/曝光{change_camera_params, camera_name, request}
- 本打算用server/client搞的，但感觉相机节点没什么好返回的，莫不如简单一点
