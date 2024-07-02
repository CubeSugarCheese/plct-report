# 准备 QEMU 环境
## 编译安装 QEMU
由于通过 apt 安装的 QEMU 版本较低，需要手动从源码安装
QEMU 自 7.2 起，不再源码内置 slirp 库（提供用户模式网络支持），需要手动安装

### 构建 libslirp
```bash
git clone https://gitlab.freedesktop.org/slirp/libslirp.git
cd libslirp
meson build
ninja -C build install
```
### 构建 QEMU
```bash
wget https://download.qemu.org/qemu-9.0.1.tar.xz
tar xf qemu-9.0.1.tar.xz && cd qemu-9.0.1
mkdir build && cd build
../configure -enable-kvm -enable-slirp=system
make -j$(nproc)
sudo make install
```

## 配置网络环境
```bash
sudo apt-get install bridge-utils
sudo brctl addbr br0
sudo brctl addif br0 eth0
sudo apt-get install -y net-tools
sudo ifconfig eth0 0.0.0.0
```

## 启动 QEMU
openEuler 镜像默认账户  
name: root  
password: openEuler12#$

### 需要下载的资源
下载下面这些镜像放置在 ./imgs 目录并解压  
http://mirror.truenetwork.ru/openeuler/openEuler-24.03-LTS/virtual_machine_img/x86_64/openEuler-24.03-LTS-x86_64.qcow2.xz
http://mirror.truenetwork.ru/openeuler/openEuler-24.03-LTS/virtual_machine_img/aarch64/openEuler-24.03-LTS-aarch64.qcow2.xz
http://mirror.truenetwork.ru/openeuler/openEuler-24.03-LTS/virtual_machine_img/riscv64/openEuler-24.03-LTS-riscv64.qcow2.xz

#### riscv64 额外需要
`RISCV_VIRT_CODE.fd`: http://mirror.truenetwork.ru/openeuler/openEuler-24.03-LTS/virtual_machine_img/riscv64/RISCV_VIRT_CODE.fd  
`RISCV_VIRT_VARS.fd`: http://mirror.truenetwork.ru/openeuler/openEuler-24.03-LTS/virtual_machine_img/riscv64/RISCV_VIRT_VARS.fd
#### aarch64 额外需要
`initrd.img`: https://repo.openeuler.org/openEuler-24.03-LTS/OS/aarch64/images/pxeboot/initrd.img  
`vmlinuz`: https://repo.openeuler.org/openEuler-24.03-LTS/OS/aarch64/images/pxeboot/vmlinuz

### 一些通用参数的解释
参考：https://www.qemu.org/docs/master/system/qemu-manpage.html

`-nographic`: 
禁用图形界面，让 QEMU 复用当前终端  
`-m 4G`: 虚拟机使用 4G 内存  
`-smp 4,sockets=2,cores=2,threads=1`: 设置仿真的 [SMP](https://baike.baidu.com/item/%E5%AF%B9%E7%A7%B0%E5%A4%9A%E5%A4%84%E7%90%86/6274908) 系统有 2 个插槽，每个插槽有 2 个核心，每个核心支持 1 个线程，总共为 4 个 CPU  
`-drive file="$drive",id=hd0,format=qcow2`: 使用指定的镜像文件作为虚拟驱动器，指定 id 为 hd0，格式为 qcow2  
`-device e1000,netdev=net0`: 添加 e1000 网卡驱动，并指定网络设备 id 为 net0  
`-netdev user,id=net0,hostfwd=tcp::"$ssh_port"-:22`: 配置用户模式网络，设备 id 与上面参数中的相同，并把虚拟机的 22 端口转发到宿主机的指定端口，便于 ssh 连接调试  
`-nic user,model=e1000,hostfwd=tcp::$ssh_port-:22`: 与以上两个参数同时使用等效，可以缩短命令长度  
`-M virt`: 指定使用 virt 平台，对于 aarch64 和 riscv64，必须指定该参数，`-M` 与 `-machine` 相同



### x86_64
```bash
sudo qemu-system-x86_64 \
  -nographic \
  -m "$memory"G \
  -smp "$vcpu",sockets=2,cores=2,threads=1 \
  -cpu host \
  -drive file="$drive",id=hd0,format=qcow2 \
  -device e1000,netdev=net0 \
  -netdev user,id=net0,hostfwd=tcp::"$ssh_port"-:22 \
  -enable-kvm
```
`-cpu host`: 选择支持当前主机 CPU 支持的所有特性的虚拟 CPU  
`-enable-kvm`: 启用 kvm 支持，提高虚拟机性能
### aarch64
```bash
qemu-system-aarch64 \
  -nographic -M virt -cpu cortex-a57 \
  -smp $vcpu -m "$memory"G \
  -hda "$drive" \
  -kernel ./vmlinuz \
  -initrd ./initrd.img \
  -nic user,model=e1000,hostfwd=tcp::$ssh_port-:22 \
  -append 'root=/dev/vda2 console=ttyAMA0'
```
`-kernel ./vmlinuz`: 指定内核镜像文件  
`-initrd ./initrd.img`: 指定文件作为初始 ram 磁盘  
`-append 'root=/dev/vda2 console=ttyAMA0'`: 向内核传递启动参数，指定根文件系统位于 /dev/vda2，控制台输出被重定向到 ttyAMA0。

### riscv64
选自 openEuler 官方提供的启动脚本
```bash
qemu-system-riscv64 \
  -nographic -machine virt,pflash0=pflash0,pflash1=pflash1,acpi=off \
  -smp "$vcpu" -m "$memory"G \
  -object memory-backend-ram,size=4G,id=ram1 \
  -numa node,memdev=ram1 \
  -object memory-backend-ram,size=4G,id=ram2 \
  -numa node,memdev=ram2 \
  -blockdev node-name=pflash0,driver=file,read-only=on,filename="$fw1" \
  -blockdev node-name=pflash1,driver=file,filename="$fw2" \
  -drive file="$drive",format=qcow2,id=hd0 \
  -object rng-random,filename=/dev/urandom,id=rng0 \
  -device virtio-vga \
  -device virtio-rng-device,rng=rng0 \
  -device virtio-blk-device,drive=hd0,bootindex=1 \
  -device virtio-net-device,netdev=usernet \
  -netdev user,id=usernet,hostfwd=tcp::"$ssh_port"-:22 \
  -device qemu-xhci -usb -device usb-kbd -device usb-tablet
```
