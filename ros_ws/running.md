## Step 1: 

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 world map
```

## Step 2: 

```bash
colcon build
source /opt/ros/foxy/setup.bash
source ~/dev/research/GaussianBeliefTrees/ros_ws/install/setup.bash
ros2 run agent_visualizer timer_callback
```

## Step 3:
```bash
ros2 run rviz2 rviz2
```