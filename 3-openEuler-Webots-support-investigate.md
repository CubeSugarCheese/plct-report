# Webots 在 openEuler 系统下的支持度及移植调查
[参考资料](https://github.com/cyberbotics/webots_ros2/wiki)

# x86_64 平台
已经根据 [环境配置](https://github.com/CubeSugarCheese/plct-report/blob/main/2-openEuler-ROS2-Humble-test.md) 配置好了 openEuler ROS2 Humble
## 运行 Demo 测试
```bash
ros2 launch webots_ros2_universal_robot multirobot_launch.py
```
![Test 1](./pictures/webots-test-1.png)

```bash
ros2 launch webots_ros2_mavic robot_launch.py
```
![Test 2](./pictures/webots-test-2.png)

## 初步测试结论
Webots 可以在 openEuler x86_64 上正常运行，但由于虚拟机性能不够，软件运行卡顿，难以进一步测试，考虑使用物理机进行测试