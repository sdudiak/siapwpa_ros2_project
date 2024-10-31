# README

Build packages:

```
cd ~/ros2_ws
colcon build
```

Build once (after this command, Python changes will work without rebuilding)
```
cd ~/ros2_ws
colcon build --symlink-install
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
