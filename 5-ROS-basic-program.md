# ROS基础编程测试题目
- 启动小乌龟仿真界面
- 新建功能包，编写第一个ros2节点实现话题发布功能，通过发布Twist消息让小乌龟跑一个长方形轨迹（基于odom信息反馈）
- 编写第二个ros2节点实现服务调用功能：即将下列命令行实现的效果改为编程调用python服务实现 `ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"` 

## 环境配置
参考 [Configuring ROS2 Environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
和 [Introducing Turtlesim](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)

## 创建工作区
```shell
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name rectangle test_py
```

## 任务1
编写 `~/ros2_ws/src/test_py/test_py/rectangle.py`
```python
from time import sleep

import rclpy
from rclpy.node import Node

from rclpy.publisher import Publisher
from rclpy.timer import Timer
from geometry_msgs.msg import Twist, Vector3


class RectanglePublisher(Node):
    publisher: Publisher
    timer: Timer
    i: int

    def __init__(self):
        super().__init__("rectangle_publisher")  # type: ignore
        self.publisher = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist(angular=Vector3(x=0.0, y=0.0, z=0.0), linear=Vector3(x=1.0, y=0.0, z=0.0))
        self.publisher.publish(msg)
        sleep(1.5)
        msg = Twist(angular=Vector3(x=0.0, y=0.0, z=1.57), linear=Vector3(x=0.0, y=0.0, z=0.0))
        self.publisher.publish(msg)
        sleep(1.5)
        self.i += 1


def main(args: list[str] | None = None):
    rclpy.init(args=args)
    publisher = RectanglePublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

```

构建并创建节点
```shell
# 终端1
cd ~/ros2_ws/ && colcon build && source install/local_setup.sh
ros2 run turtlesim turtlesim_node
# --------------------------------
# 终端2
cd ~/ros2_ws/ && source install/local_setup.sh
ros2 run test_py rectangle
```

## 任务2
编写 `~/ros2_ws/src/test_py/test_py/call_spawn.py`

```python
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.task import Future

from turtlesim.srv import Spawn

class CallSpawn(Node):
    fut: Future
    
    def __init__(self):
        super().__init__("call_spawn")  # type: ignore
        
        client = self.create_client(Spawn, "/spawn")
        self.fut = client.call_async(Spawn.Request(x = 2.0, y = 2.0, theta = 0.2, name = ''))


def main(args: list[str] | None = None):
    rclpy.init(args=args)
    call_spawn = CallSpawn()
    rclpy.spin_until_future_complete(call_spawn, call_spawn.fut)
    call_spawn.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```
需要更新 `~/ros2_ws/src/test_py/setup.py`

```python
    ...
    entry_points={
        'console_scripts': [
            'rectangle = test_py.rectangle:main',
            'call_spawn = test_py.call_spawn:main',
        ],
    },
    ...
```

构建并创建节点
```shell
# 终端1
cd ~/ros2_ws/ && colcon build && source install/local_setup.sh
ros2 run turtlesim turtlesim_node
# --------------------------------
# 终端2
cd ~/ros2_ws/ && source install/local_setup.sh
ros2 run test_py call_spawn
```