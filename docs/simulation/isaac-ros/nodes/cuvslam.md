# isaac_ros_visual_slam (cuVSLAM)

GPU-accelerated visual SLAM using stereo cameras. Estimates the robot's pose and builds a map of visual landmarks.

Uses a custom launch script at `~/workspaces/isaac_ros-dev/launch_cuvslam.sh`:

```bash
docker exec -it isaac_ros_dev_container bash /workspaces/isaac_ros-dev/launch_cuvslam.sh
```

| | |
|---|---|
| **Launch script** | `launch_cuvslam.sh` (custom — NVIDIA's default launch file doesn't expose TF params) |
| **Subscribes to** | `/front_stereo_camera/left/image_rect_color`, `/right/image_rect_color`, and their `camera_info` topics |
| **Publishes** | `/visual_slam/tracking/odometry`, `/visual_slam/vis/landmarks_cloud`, `/visual_slam/tracking/slam_path` |
| **Foxglove topics** | Add `landmarks_cloud` (decay=0), `slam_path`, and `observations_cloud` to the 3D panel |

## TF ownership and the coordinate system

The ROS TF chain is `map → odom → base_link`. On a real robot, SLAM owns this entire chain — there's no simulator giving ground-truth odometry.

In simulation, Isaac Sim and cuVSLAM both want to publish `odom → base_link`, but they compute different poses (Isaac Sim uses perfect physics, cuVSLAM uses visual feature tracking). If both publish, the lidar point cloud flickers between the two competing poses in Foxglove.

**The fix has two parts:**

1. **Isaac Sim side** — the headless launch script (`headless-sample-scene.sh`) disables Isaac Sim's `odom → base_link` TF publisher via the OmniGraph API:

    ```python
    import omni.graph.core as og
    graph = og.get_graph_by_path("/World/Nova_Carter_ROS/transform_tree_odometry")
    node = graph.get_node(".../ros2_publish_raw_transform_tree")
    node.set_disabled(True)
    ```

2. **cuVSLAM side** — the custom launch script sets `publish_odom_to_base_tf: True` and `publish_map_to_odom_tf: True` so cuVSLAM is the sole authority.

This matches real-robot behavior: cuVSLAM owns the full TF chain, and the lidar and SLAM landmarks render in the same coordinate system.

!!! note "cuVSLAM TF params must be set at launch time"
    cuVSLAM caches `publish_odom_to_base_tf` and `publish_map_to_odom_tf` as C++ member variables at startup. Using `ros2 param set` at runtime has no effect — the node never re-reads them. Always set these in the launch file's `parameters` dict.

!!! warning "Stopping on-demand nodes"
    Killing a tmux session running `docker exec` does **not** kill the process inside the container — it only kills the exec client on the host. The node keeps running and will conflict if you launch another instance. Always stop container processes from inside:

    ```bash
    docker exec isaac_ros_dev_container bash -c "pkill -f visual_slam"
    ```
