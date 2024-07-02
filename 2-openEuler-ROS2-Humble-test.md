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
TODO

# 测试结果
## x86_64
TODO
## aarch64
TODO
## riscv64
TODO

# 遇到的一些问题
## 安装软件包时 curl 提示 SSL 证书验证失败
```
openEulerROS-humble':
  - Curl error (60): SSL peer certificate or SSH remote key was not OK for https://build-repo.tarsier-infra.isrc.ac.cn/openEuler:/ROS/24.03/repodata/repomd.xml [SSL]
Error: Failed to download metadata for repo 'openEulerROS-humble': Cannot download repomd.xml: Cannot download repodata/repomd.xml: All mirrors were tried`
```
经检查，是系统时间问题。通过 ntp 重新同步虚拟机内系统时间后解决。