# Webots 基础教程
## 教程目标
本教程旨在介绍通过 `ros2 hubmle` 提供的框架与 `Webots` 交互的方法。对于 `Webots` 本体的内容不做过多介绍，
如果有兴趣， 请参阅[Webots User Guide](https://cyberbotics.com/doc/guide/introduction-to-webots)

## 环境部署
本教程假设你已经安装好了 `ros2 humble` 以及 `Webots`，并正确设置了环境变量 `WEBOTS_HOME`

如果没有，请参考 [ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation.html) 和 [Webots Installation (Ubuntu)](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html)

此外，你还需要创建好 ros2 的工作区，请参见[创建工作区](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#create-a-workspace)

为了更好的自动补全，需要将 `/opt/ros/humble/local/lib/python3.10/dist-packages` 添加到 python 包搜索目录。
例如，如果你使用 `vscode` 的 python 插件，你需要在 `settings.json` 中添加以下内容
```json
    "python.analysis.extraPaths": [
        "/opt/ros/humble/local/lib/python3.10/dist-packages"
    ],
    "python.autoComplete.extraPaths": [
        "/opt/ros/humble/local/lib/python3.10/dist-packages"
    ],
```


## 原理介绍
`webots_ros2` 通过[控制器插件](https://cyberbotics.com/doc/guide/controller-plugin)与 `Webots` 交互

## 基础示例
以下内容基于 [设置基本 Webots 模拟](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html)

```shell
# 在工作区的 src 目录（~/ros2_ws/src）执行命令
ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name my_robot_driver my_package --dependencies rclpy geometry_msgs webots_ros2_driver
cd my_package
mkdir launch
mkdir worlds
```

下载这个示例 wbt 文件，存储到 `src/my_package/worlds/my_world.wbt` https://docs.ros.org/en/humble/_downloads/5ad123fc6a8f1ea79553d5039728084a/my_world.wbt


`src/my_package/resource/my_robot.urdf`  
URDF（统一机器人描述格式）是一种用于指定 ROS 中机器人的几何形状和组织结构的文件格式。对于 URDF 文件的更多信息，请参见 [URDF Main](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
```xml
<?xml version="1.0" ?>
<robot name="My robot">
    <webots>
        <plugin type="my_package.my_robot_driver.MyRobotDriver" />
    </webots>
</robot>
```

`src/my_package/launch/robot_launch.py`  
ROS 2 启动文件用于方便的同时启动和配置多个包含 ROS 2 节点的可执行文件。详细信息请参考 [Launch Main](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
```python
import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description() -> LaunchDescription:
    package_dir = get_package_share_directory("my_package")
    robot_description_path = os.path.join(package_dir, "resource", "my_robot.urdf")

    webots = WebotsLauncher(world=os.path.join(package_dir, "worlds", "my_world.wbt"))

    my_robot_driver = WebotsController(
        robot_name="my_robot",
        parameters=[
            {"robot_description": robot_description_path},
        ],
    )

    return LaunchDescription(
        [
            webots,
            my_robot_driver,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=webots,
                    on_exit=[EmitEvent(event=Shutdown())],
                )
            ),
        ]
    )
```

`src/my_package/my_package/my_robot_driver.py`:
```python
import rclpy
from geometry_msgs.msg import Twist
from controller import Robot, Motor


HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot: Robot = webots_node.robot

        self.__left_motor: Motor = self.__robot.getDevice('left wheel motor') # type: ignore
        self.__right_motor: Motor = self.__robot.getDevice('right wheel motor') # type: ignore

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver') # type: ignore
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)
```

修改 `setup.py`
```python
package_name = 'my_package'
data_files: list[tuple[str, list[str]]] = []
data_files.append(('share/ament_index/resource_index/packages', [f'resource/{package_name}']))
data_files.append((f'share/{package_name}/launch', ['launch/robot_launch.py']))
data_files.append((f'share/{package_name}/worlds', ['worlds/my_world.wbt']))
data_files.append((f'share/{package_name}/resource', ['resource/my_robot.urdf']))
data_files.append((f'share/{package_name}', ['package.xml']))

setup(
    ...,
    data_files=data_files,
    ...
)
```

### 运行
在工作区目录（`~/ros2_ws`）中运行
```shell
colcon build
source install/local_setup.bash/local_setup
ros2 launch my_package robot_launch.py
```
Webots 会自动启动

在另一个终端运行以下命令
```shell
ros2 topic pub /cmd_vel geometry_msgs/Twist  "linear: { x: 0.1 }"
```
可以看到 Webots 中的机器人开始前进直到撞墙

## 进阶示例
以下内容基于 [设置进阶 Webots 模拟](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html)

基础示例中编写的机器人会傻傻的撞到墙上，在进阶示例中，我们会通过传感器获取信息，并为机器人提供避障功能。

更新 `my_robot.urdf`
```xml
<?xml version="1.0" ?>
<robot name="My robot">
    <webots>
        <device reference="ds0" type="DistanceSensor">
            <ros>
                <topicName>/left_sensor</topicName>
                <alwaysOn>true</alwaysOn>
            </ros>
        </device>
        <device reference="ds1" type="DistanceSensor">
            <ros>
                <topicName>/right_sensor</topicName>
                <alwaysOn>true</alwaysOn>
            </ros>
        </device>
        <plugin type="my_package.my_robot_driver.MyRobotDriver" />
    </webots>
</robot>
```

`src/my_package/my_package/obstacle_avoider.py`:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist


MAX_RANGE = 0.15


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider') # type: ignore

        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        self.create_subscription(Range, 'left_sensor', self.__left_sensor_callback, 1)
        self.create_subscription(Range, 'right_sensor', self.__right_sensor_callback, 1)

    def __left_sensor_callback(self, message: Range):
        self.__left_sensor_value = message.range

    def __right_sensor_callback(self, message: Range):
        self.__right_sensor_value = message.range

        command_message = Twist()

        command_message.linear.x = 0.1

        if self.__left_sensor_value < 0.9 * MAX_RANGE or self.__right_sensor_value < 0.9 * MAX_RANGE:
            command_message.angular.z = -2.0

        self.__publisher.publish(command_message)


def main(args=None):
    rclpy.init(args=args)
    avoider = ObstacleAvoider()
    rclpy.spin(avoider)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

更新 `setup.py`
```python
setup(
    ...
    'console_scripts': [
        'my_robot_driver = my_package.my_robot_driver:main',
        'obstacle_avoider = my_package.obstacle_avoider:main'
    ],
    ...
)
```

更新 `robot_launch.py​​`
```python
import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description() -> LaunchDescription:
    package_dir = get_package_share_directory("my_package")
    robot_description_path = os.path.join(package_dir, "resource", "my_robot.urdf")

    webots = WebotsLauncher(world=os.path.join(package_dir, "worlds", "my_world.wbt"))

    my_robot_driver = WebotsController(
        robot_name="my_robot",
        parameters=[
            {"robot_description": robot_description_path},
        ],
    )
    
    obstacle_avoider = Node(
        package='my_package',
        executable='obstacle_avoider',
    )

    return LaunchDescription(
        [
            webots,
            my_robot_driver,
            obstacle_avoider,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=webots,
                    on_exit=[EmitEvent(event=Shutdown())],
                )
            ),
        ]
    )
```

### 运行
在工作区目录（`~/ros2_ws`）中运行
```shell
colcon build
source install/local_setup.sh
ros2 launch my_package robot_launch.py
```
Webots 会自动启动

可以看到 Webots 中的机器人开始前进并在快撞墙的时候右转