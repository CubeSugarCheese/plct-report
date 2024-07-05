# 配置 openEuler ROS2 Humble  环境
## 添加软件源
创建 `/etc/yum.repos.d/openEulerROS.repo` 文件并写入以下内容
```
[openEulerROS-humble]
name=openEulerROS-humble
baseurl={{repo_url}}
enabled=1
gpgcheck=0
```
各平台对应的仓库地址如下：  
`x86_64`: `https://eulermaker.compass-ci.openeuler.openatom.cn/api/ems1/repositories/ROS-SIG-Multi-Version_ros-humble_openEuler-24.03-LTS-TEST4/openEuler%3A24.03-LTS/x86_64/`  

`aarch64`: `https://eulermaker.compass-ci.openeuler.openatom.cn/api/ems1/repositories/ROS-SIG-Multi-Version_ros-humble_openEuler-24.03-LTS-TEST4/openEuler%3A24.03-LTS/aarch64/`  

`riscv64`: `https://build-repo.tarsier-infra.com/openEuler:/ROS/24.03/`
## 安装软件包
```bash
dnf install "ros-humble-*" --skip-broken --exclude=ros-humble-generate-parameter-library-example
```

## 添加环境变量
```bash
echo "source /opt/ros/humble/setup.sh" >> ~/.bashrc
source ~/.bashrc
```

## 配置 XFCE 桌面环境
[参考资料](https://docs.openeuler.org/en/docs/20.03_LTS_SP2/docs/desktop/installing-Xfce.html)
### 安装 XFCE
```bash
sudo dnf update
# 安装字体库
sudo dnf install dejavu-fonts liberation-fonts gnu-*-fonts google-*-fonts
# 安装Xorg
sudo dnf install xorg-*
# 安装 Xfce 及组件
sudo dnf install xfwm4 xfdesktop xfce4-* xfce4-*-plugin network-manager-applet *fonts
```
### 配置登录管理器
```bash
sudo dnf install lightdm lightdm-gtk
# 设置默认桌面为 Xfce
echo 'user-session=xfce' >> /etc/lightdm/lightdm.conf.d/60-lightdm-gtk-greeter.conf
sudo systemctl start lightdm
sudo systemctl enable lightdm
sudo systemctl set-default graphical.target
# 如果安装了 gdm，需要执行下面这行
# systemctl disable gdm
```
至此，桌面环境安装完毕，重启 QEMU 并去除 `-nographic` 参数即可。

# 测试流程
## 测试环境
### 硬件信息
CPU 13th Gen Intel i9-13900H  
内存 32GB
### 软件信息
宿主机 OS：Kubuntu 22.04.4 LTS x86_64  
虚拟机 OS：openEuler-24.03-[x86/aarch64/riscv64]

## turtlesim（需要桌面环境）
```bash
ros2 run turtlesim turtlesim_node # 终端1
# 出现模拟器窗口，中间有一只随机的海龟
ros2 run turtlesim turtle_teleop_key # 终端2

```
## pkg
```bash
ros2 pkg create test-pkg
ros2 pkg exectables
ros2 pkg list
ros2 pkg prefix turtlesim
ros2 pkg xml turtlesim
```

## topic
```bash
ros2 topic list
ros2 topic info /rosout
ros2 topic type /rosout
ros2 topic find rcl_interfaces/msg/Log

ros2 run demo_nodes_cpp talker # 终端1
ros2 topic hz /chatter # 终端2

ros2 run demo_nodes_cpp talker # 终端1
ros2 topic bw /chatter # 终端2

ros2 run demo_nodes_cpp talker # 终端1
ros2 topic echo /chatter # 终端2

ros2 run demo_nodes_cpp talker # 终端1
ros2 topic param /chatter # 终端2

ros2 run demo_nodes_cpp talker # 终端1
ros2 topic service /chatter # 终端2

```

## node
```bash
ros2 run demo_nodes_cpp talker # 终端1
ros2 node list /chatter # 终端2

ros2 run demo_nodes_cpp talker # 终端1
ros2 node info /chatter # 终端2
```

## bag
```bash
ros2 bag record -a
# 这里是上一步中看到的文件名
ros2 bag info rosbag2_2024_07_05-06_05_07/rosbag2_2024_07_05-06_05_07_0.db3
ros2 bag play rosbag2_2024_07_05-06_05_07/rosbag2_2024_07_05-06_05_07_0.db3
```

## launch
```bash
ros2 launch demo_nodes_cpp talker_listener.launch.py
```
# 测试结果
✅成功  
❌失败  
⛔未测试
## x86_64
测试功能   | 测试结果
----------|--------
turtlesim | ✅成功
pkg       | ✅成功
topic     | ✅成功
node      | ✅成功
bag       | ✅成功
launch    | ✅成功

## aarch64
测试功能   | 测试结果
----------|--------
turtlesim | ⛔未测试
pkg       | ⛔未测试
topic     | ⛔未测试
node      | ⛔未测试
bag       | ⛔未测试
launch    | ⛔未测试
## riscv64
测试功能   | 测试结果
----------|--------
turtlesim | ⛔未测试
pkg       | ⛔未测试
topic     | ⛔未测试
node      | ⛔未测试
bag       | ⛔未测试
launch    | ⛔未测试

# 遇到的一些问题
## 安装软件包时 curl 提示 SSL 证书验证失败
```
openEulerROS-humble':
  - Curl error (60): SSL peer certificate or SSH remote key was not OK for https://build-repo.tarsier-infra.isrc.ac.cn/openEuler:/ROS/24.03/repodata/repomd.xml [SSL]
Error: Failed to download metadata for repo 'openEulerROS-humble': Cannot download repomd.xml: Cannot download repodata/repomd.xml: All mirrors were tried`
```
经检查，是系统时间问题。通过 ntp 重新同步虚拟机内系统时间后解决。