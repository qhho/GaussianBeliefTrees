
# Running the Visualization with RViz and ROS2

Follow these steps to set up and run your visualization using RViz in ROS2.

### **Step 1: Publish a Static Transform**

First, we need to publish a static transform between the `world` and `map` frames. Open a new terminal window and enter the following command:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 world map
```

This command publishes a static transform with no translation or rotation between the `world` and `map` frames.

### **Step 2: Build the Workspace and Run the Visualizer**

Next, we'll build the ROS workspace and run the visualizer node.

1. Open a **new terminal window**.
2. Navigate to the `GaussianBeliefTrees/ros_ws` directory:

    ```bash
    colcon build
    source /opt/ros/foxy/setup.bash
    source ~/dev/research/GaussianBeliefTrees/ros_ws/install/setup.bash
    ros2 run agent_visualizer timer_callback
    ```

### **Step 3: Launch RViz**

Finally, open a new terminal and launch RViz:

```bash
ros2 run rviz2 rviz2
```

In RViz:
- Click `Add` in the bottom left.
- Add a MarkerArray and change the topic to `visualization_marker_array/obstacles`.
- Add another MarkerArray and change the topic to `visualization_marker_array/agent`.
