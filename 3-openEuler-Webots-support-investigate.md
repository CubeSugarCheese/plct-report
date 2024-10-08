# Webots 在 openEuler 系统下的支持度及移植调查
[参考资料](https://github.com/cyberbotics/webots_ros2/wiki)

## Webots 简介
[官网](https://cyberbotics.com/)  [仓库](https://github.com/cyberbotics/webots) [文档](https://cyberbotics.com/doc/guide/menu) 

Webots 是一款免费，开源的 3D 机器人模拟软件，可以模拟各种机器人，包括两轮机器人、工业手臂、腿式机器人、模块化机器人、汽车、无人机、自主水下航行器、履带式机器人、航空航天器等。  
Webots 核心基于现代 GUI 框架 Qt、物理引擎 (ODE fork) 和 OpenGL 3.3 渲染引擎 (wren) 的组合。它可以在 Windows、Linux 和 macOS 上运行。

## openEuler 软件包引入检查
|    检查点    | 说明 | 检查结果 |
|--------------|------|----------|
|归一化        |一款软件只在src-openeuler中引入一次|✅尚未在 gitee.com/src-openeuler 下查询到对应仓库|
|来源可靠      |开源软件应从官网或官网指定的代码托管地址获取|✅github.com/cyberbotics/webots 为官方代码托管地址|
|规范化软件名称|软件名称需要和官网/社区保持一致|✅与官方版本发布页面的命名相同 github.com/cyberbotics/webots/releases|
|社区运营状态  |停止维护或维护状态不明确的，不建议引入|✅查看半年内的commits记录和issues，pr记录，可以认为处于积极维护中|
|官网          |软件官网需要填写规范，不使用maven等托管库作为官网|✅软件官网为 cyberbotics.com|
|软件包信息提供|必须提供官方提供的源码下载地址，如有二进制包需要，也必须提供官方提供的二进制包下载地址|✅源码包地址为官方提供|
|License检查   |入库时填写的License信息需要和官网，软件包内保持一致，高风险License需要谨慎考虑引入|✅Webots使用Apache-2.0开源许可|
|Copyright检查 |通过官网、社区、代码托管网站、源码包、发布件中等获取并提供Copyright信息|✅Copyright 1996-2023 Cyberbotics Ltd. github.com/cyberbotics/webots/blob/master/LICENSE|


## x86_64 原生构建及打包
[参考资料](https://github.com/cyberbotics/webots/wiki/Linux-installation/)
### 下载源码
```bash
git clone --recurse-submodules -j8 https://github.com/cyberbotics/webots.git
cd ./webots
```
### 依赖安装
```bash
dnf install openeuler-lsb cmake swig mesa-libGL glib2-devel freeimage-devel freetype-devel libxml2-devel boost-devel libssh-devel libzip readline-devel pbzip2 wget zip unzip glm-devel stb_image-devel stb_image_write-devel
```
### 构建
这一步需要良好的网络环境
```bash
make -j8
```
构建完成后直接运行项目目录下的 `webots` sh 脚本即可，需要在桌面环境下运行。
### 设置环境变量
```
export WEBOTS_HOME={webots_dir}
```

### 打包成 rpm
参见 [openEuler 系统下的 rpm 打包教程，以 Webots 为例](./4-openEuler-create-rpm-pack.md)

## 测试运行
已经根据 [环境配置](https://github.com/CubeSugarCheese/plct-report/blob/main/2-openEuler-ROS2-Humble-test.md) 配置好了 openEuler ROS2 Humble

### 运行 Demo 测试
初次运行命令时如果没有设置 Webots 目录对应的环境变量会提示是否自动安装 Webots，选择是后会自动下载最新稳定版本并设置环境变量，从源码安装的是 nightly 版本，二者版本不同为正常现象。

[Webots 版本发布页面](https://github.com/cyberbotics/webots/releases)
```bash
ros2 launch webots_ros2_universal_robot multirobot_launch.py
```
![Test 1](./pictures/webots-test-1.png)

```bash
ros2 launch webots_ros2_mavic robot_launch.py
```
![Test 2](./pictures/webots-test-2.png)

### 初步测试结论
Webots 可以在 openEuler x86_64 上正常运行，但由于虚拟机性能不够，软件运行卡顿，难以进一步测试，考虑使用物理机进行测试