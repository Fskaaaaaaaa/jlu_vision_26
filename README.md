# CCUPT2026_infantry_vision  

一个基于iceoryx零拷贝通信的robomaster自瞄框架

## 目录

- [上手指南](#上手指南)
  - [开发前的配置要求](#开发前的配置要求)
  - [安装步骤](#安装步骤)
- [文件目录说明](#文件目录说明)
- [开发的架构](#开发的架构)
- [部署](#部署)
- [使用到的框架](#使用到的框架)
- [贡献者](#贡献者)
  - [如何参与开源项目](#如何参与开源项目)
- [版本控制](#版本控制)
- [作者](#作者)
- [鸣谢](#鸣谢)


### 文件目录说明
```
filetree 
├── assets  运行时依赖
├── configs  配置文件
├── doc  文档
├── src
│   ├── auto_aim  自瞄相关组件
│   │   ├── armor_detector
│   │   ├── armor_tracker
│   │   └── xmake.lua
│   ├── auto_ballistic  弹道闭环控制组件
│   │   ├── armor_corrector
│   │   ├── bullet_detector
│   │   └── xmake.lua
│   ├── auto_buff  打符
│   │   └── xmake.lua
│   ├── hardware  硬件通信相关组件
│   │   ├── camera
│   │   ├── gimbal_planner
│   │   ├── serial
│   │   └── xmake.lua
│   ├── msgs  iceoryx通信用到的消息结构体定义
│   │   ├── msgs
│   │   └── xmake.lua
│   ├── third_party  xmake缺失的第三方依赖
│   │   ├── fast_tf
│   │   ├── hik_camera_sdk
│   │   ├── open3d
│   │   ├── tinympc
│   │   └── xmake.lua
│   └── tools 一些简单的工具
│       ├── audible_alarm
│       ├── basic
│       ├── math
│       ├── visualization
│       └── xmake.lua
├── LICENSE.txt
├── README.md
└── xmake.lua
```

### 部署

暂无

### 使用到的框架

- [xxxxxxx](https://getbootstrap.com)
- [xxxxxxx](https://jquery.com)
- [xxxxxxx](https://laravel.com)

### 作者

xxx@xxxx

知乎:xxxx  &ensp; qq:xxxxxx    

 *您也可以在贡献者名单中参看所有参与该项目的开发者。*

### 版权说明

该项目签署了MIT 授权许可，详情请参阅 LICENSE.txt

### 鸣谢


- [GitHub Emoji Cheat Sheet](https://www.webpagefx.com/tools/emoji-cheat-sheet)
- [Img Shields](https://shields.io)
- [Choose an Open Source License](https://choosealicense.com)
- [GitHub Pages](https://pages.github.com)
- [Animate.css](https://daneden.github.io/animate.css)
- [xxxxxxxxxxxxxx](https://connoratherton.com/loaders)
