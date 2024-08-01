# openEuler 系统下的 rpm 打包教程，以 Webots 为例
## 本地打包
### 打包环境
系统版本：openEuler
### 准备打包环境
```shell
# 安装打包工具
dnf install rpmdevtools
# 创建打包工作目录
rpmdev-setuptree
# 默认情况下会在 /root （普通用户为 /home/{username}）下创建 rpmbuild 文件夹
# rpmbuild
# ├── BUILD
# ├── RPMS
# ├── SOURCES
# ├── SPECS
# └── SRPMS
```
#### 目录说明
| 目录 | 宏代码 | 名称 | 功能 |
-------|--------|------|-------
| BUILD | %_builddir | 构建目录 | 源码包被解压至此，并在该目录的子目录完成编译 |
| RPMS | %_rpmdir  | 标准 RPM 包目录 | 生成/保存二进制 RPM 包 |
| SOURCES | %_sourcedir | 源代码目录 | 保存源码包（如 .tar 包）和所有 patch 补丁 |
| SPECS | %_specdir | Spec 文件目录 | 保存 RPM 包配置（.spec）文件 |
| SRPMS | %_srcrpmdir | 源代码 RPM 包目录 | 生成/保存源码 RPM 包(SRPM) |
#### 打包流程
1. 把源代码放到 `%_sourcedir` 中。
2. 进行编译，编译的过程是在 `%_builddir` 中完成的，一般情况下，源代码是压缩包格式，需要先进行解压。
进行“安装”，类似于预先组装软件包，把软件包应该包含的内容（比如二进制文件、配置文件、man文档等）复制到 `%_buildrootdir` 中，并按照实际安装后的目录结构组装，比如二进制命令可能会放在 `/usr/bin` 下，那么就在 `%_buildrootdir` 下也按照同样的目录结构放置。
3. 做一些必要的配置，比如在实际安装前的准备，安装后的清理等等。这些都是通过配置在SPEC文件中来告诉rpmbuild命令。
4. 检查软件是否正常运行。
5. 生成的RPM包放置到 `%_rpmdir`，源码包放置到 `%_srcrpmdir` 下，这一步由打包工具自动进行。
### 编写 spec 文件
```
Name:     Webots
Version:  R2023b
Release:  1%{?dist}
Summary:  Open-source robot simulator
Summary(zh_CN):  开源机器人模拟器
License:  Apache-2.0
URL:      https://github.com/cyberbotics/webots
Source0:  https://github.com/cyberbotics/webots/archive/refs/tags/R2023b.tar.gz
# 下面这些是在 make build 过程中自动下载到项目目录的，为了在离线环境中打包，需要手动准备这些依赖
Source1:  https://cyberbotics.com/files/repository/dependencies/linux64/release/webots-qt-6.4.3-linux64-release.tar.bz2
Source2:  https://cyberbotics.com/files/repository/dependencies/linux64/release/openal-linux64-1.16.0.tar.bz2
Source3:  https://cyberbotics.com/files/repository/dependencies/linux64/release/libOIS.1.4.tar.bz2
Source4:  https://cyberbotics.com/files/repository/dependencies/linux64/release/libassimp-5.2.3.tar.bz2
Source5:  https://cyberbotics.com/files/repository/dependencies/linux64/release/libpico.tar.bz2
Source6:  https://www.lua.org/ftp/lua-5.2.3.tar.gz

BuildRequires:  gcc,make,git,Xvfb,openeuler-lsb,cmake,swig,mesa-libGL,freeimage,freetype-devel,libxml2-devel,boost-devel,libssh-devel,libzip,readline-devel,pbzip2,wget,zip,unzip,glm-devel,stb_image-devel,stb_image_write-devel,glibc-devel
BuildArch:      x86_64
# 让 rpmbuild 自动扫描依赖会报 glibc 版本错误，需要手动编写依赖
AutoReqProv: no


%description
Webots provides a complete development environment to model, program and simulate robots, vehicles and mechanical systems.

%description -l zh_CN
Webots提供了一个完整的开发环境来建模、编程和模拟机器人、车辆和机械系统。

# 禁止生成 debug 子包
%define  debug_package %{nil}
# 禁用 rpath 和 buildroot 检查
%define __arch_install_post %{nil}

%prep
%autosetup -n webots-%{version} -a 1 -a 2 -a 3 -a 4 -a 5 -a 6
# 创建dependencies目录
mkdir -p dependencies
# 移动依赖文件
cp %{SOURCE1} dependencies/
cp %{SOURCE3} dependencies/
cp %{SOURCE4} dependencies/
cp %{SOURCE5} dependencies/
# 解压需要解压的依赖文件到dependencies目录
tar -xjf %{SOURCE2} -C dependencies/
tar -xzf %{SOURCE6} -C dependencies/

%build
make %{?_smp_mflags}

%install
mkdir -p %{buildroot}/opt/%{name}-%{version}
cp -a * %{buildroot}/opt/%{name}-%{version}/

%files
%license LICENSE
%doc README.md
%defattr(-,root,root,-)
%dir /opt/%{name}-%{version}
/opt/%{name}-%{version}/*
```
### 进行构建
| :warning: 注意 |
|:----------------------------|
| Webots 是一个大体量的软件，构建过程中会占用大量内存和 cpu，并且构建时间较长 |
在开始构建前，需要手动下载 Source{N} 链接指向的文件并放在 SOURCES 目录，rpmbuild 不会自动下载这些文件。
```shell
# 从头开始构建
rpmbuild -ba {specfile}
# 对于编写 spec 过程中可能会频繁进行某个特定阶段
# 调试的需求，可以使用以下命令从特定阶段开始构建
# -b参数中 p/c/i/l 分别代表了
# %prep/%build/%install/%files 阶段
rpmbuild --short-circuit -b{p/c/i/l} {specfile}
```
打包成功后会在 PRM 目录生成对应 rpm 文件。
### 测试软件包
```
dnf install ~/rpmbuild/RPMS/x86_64/Webots-R2023b-1.x86_64.rpm
```
安装完后应该可以看见软件已经安装在 `/opt/Webots-R2023b` 下了

## 通过 eulerMaker 打包
### 创建 Git 仓库

## 参考资料
1. [openEuler Docs：构建RPM包](https://docs.openeuler.org/zh/docs/24.03_LTS/docs/ApplicationDev/%E6%9E%84%E5%BB%BARPM%E5%8C%85.html)
2. [openEuler packaging guidelines](https://gitee.com/openeuler/community/blob/master/zh/contributors/packaging.md)
3. [rpm包的制作笔记](https://zhangguanzhang.github.io/2017/05/15/rpmbuild/#/rpm-%E4%B8%80%E4%BA%9B%E7%9F%A5%E8%AF%86)