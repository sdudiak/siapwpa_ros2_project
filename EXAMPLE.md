# Example of ROS2 packages in Python

* `package.xml` file containing meta information about the package  
* `resource/<package_name>` marker file for the package  
* `setup.cfg` is required when a package has executables, so `ros2 run` can find them  
* `setup.py` containing instructions for how to install the package  
* `<package_name>` - a directory with the same name as your package, used by ROS 2 tools to find your package, contains `__init__.py`

```bash
ros2 pkg create --build-type ament_python --node-name my_node my_package
```

> [!TIP] 
> W pliku `package.xml` należy zawsze dodać wszystkie używane biblioteki, jak w przykładzie poniżej:
> ```xml
> <exec_depend>rclpy</exec_depend>  
> <exec_depend>std_msgs</exec_depend>
> ```
>
> W pliku `setup.py` należy dodać:
> ```python
> entry_points={
>        'console_scripts': [
>                '<name_run> = <package_name>.<file_name>:<function_name>',
>        ],
>},
> ```
>
> W pliku `setup.cfg` sprawdzić czy są poprawne dane, jak poniżej:
> ```properties
> [develop]
>script_dir=$base/lib/<package_name>
>[install]
>install_scripts=$base/lib/<package_name>
> ```

## How to create simple publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    try:
      rclpy.spin(minimal_publisher)
    except KeyboardInterrupt: 
      pass
    finally:
      # Destroy the node explicitly
      # (optional - otherwise it will be done automatically
      # when the garbage collector destroys the node object)
      minimal_publisher.destroy_node()
      rclpy.shutdown()


if __name__ == '__main__':
    main()
```



## How to create simple subscriber
```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    try:
      rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt: 
      pass
    finally:
      # Destroy the node explicitly
      # (optional - otherwise it will be done automatically
      # when the garbage collector destroys the node object)
      minimal_subscriber.destroy_node()
      rclpy.shutdown()


if __name__ == '__main__':
    main() 
```

## How to create launch file [More info](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)

```bash
ros2 launch <package_name> <launch_file_name>
```

> [!TIP] 
> Unique `namespaces` allow the system to start two similar nodes without `node` name or` topic` name conflicts.

> [!NOTE] 
> For packages with `launch` files, it is a good idea to add an `exec_depend` dependency on the `ros2launch` package in your package’s `package.xml`:
> ```xml 
> <exec_depend>ros2launch</exec_depend>
> ```

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```