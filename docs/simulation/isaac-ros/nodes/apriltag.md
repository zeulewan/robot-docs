# isaac_ros_apriltag

GPU-accelerated AprilTag detection and 6DOF pose estimation.

```bash
docker exec -it isaac_ros_dev_container bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && \
   export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_no_shm.xml && \
   ros2 launch isaac_ros_apriltag isaac_ros_apriltag_isaac_sim_pipeline.launch.py"
```

| | |
|---|---|
| **Subscribes to** | `/front_stereo_camera/left/image_rect_color`, `/front_stereo_camera/left/camera_info` |
| **Publishes** | `/tag_detections` (`isaac_ros_apriltag_interfaces/AprilTagDetectionArray`) |
