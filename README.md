# README

Build packages:

```
cd ~/ros2_ws
colcon build
```

Run the simulation + rviz + bridge setup

```
source install/setup.bash
ros2 launch simple_example example.launch.py
```

Run the `track_controller_node` 
```
ros2 run simple_example track_controller
```
