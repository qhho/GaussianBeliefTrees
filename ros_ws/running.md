## Step 1: 

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 world map
```

## Step 2: 

```bash
colcon build --packages-select agent_gaussian_belief_trees
source install/setup.bash
ros2 run agent_gaussian_belief_trees obstacle_publisher
```

## Step 3:
```bash
ros2 run rviz2 rviz2
```