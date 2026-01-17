# 坐标系变换

- coord_transform组件里的动态发布和静态服务
- tools里的libtf_listener

## 关于listener

订阅话题：

- {"tf", "dynamic", "data"}

- {"tf", "static", "data"}

故需要publisher向{“tf”，“transform”，“update”}发布各类变换信息。

注意时间戳和parent_frame在Header里。

