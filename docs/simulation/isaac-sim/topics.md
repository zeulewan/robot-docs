# ROS 2 Topics (Carter Warehouse)

ROS 2 topics published by the `carter_warehouse_apriltags_worker.usd` scene in Isaac Sim, plus topics added by the Isaac ROS container.

!!! info "Active sensors"
    The Nova Carter has many sensors but only the **front stereo camera**, **front 3D lidar**, and **chassis IMU** are publishing active data topics in this scene. The remaining sensor frames exist in the TF tree but don't have corresponding data topics.

---

## Sensors

| Topic | Type | Description |
|---|---|---|
| `/front_stereo_camera/left/image_rect_color` | `sensor_msgs/Image` | Left stereo camera (rectified) |
| `/front_stereo_camera/right/image_rect_color` | `sensor_msgs/Image` | Right stereo camera (rectified) |
| `/front_stereo_camera/left/camera_info` | `sensor_msgs/CameraInfo` | Left camera calibration |
| `/front_stereo_camera/right/camera_info` | `sensor_msgs/CameraInfo` | Right camera calibration |
| `/front_3d_lidar/lidar_points` | `sensor_msgs/PointCloud2` | 3D lidar point cloud |
| `/chassis/imu` | `sensor_msgs/Imu` | Chassis IMU |
| `/front_stereo_imu/imu` | `sensor_msgs/Imu` | Front stereo camera IMU |
| `/back_stereo_imu/imu` | `sensor_msgs/Imu` | Back stereo camera IMU |
| `/left_stereo_imu/imu` | `sensor_msgs/Imu` | Left stereo camera IMU |
| `/right_stereo_imu/imu` | `sensor_msgs/Imu` | Right stereo camera IMU |

!!! note "Rectified images"
    "Rectified" means lens distortion is removed and stereo pairs are aligned so left/right pixels share the same row -- ready for depth computation. In simulation this is a formality (virtual cameras have no distortion), but the topic names match real Nova Carter camera pipelines.

---

## Robot State

| Topic | Type | Description |
|---|---|---|
| `/chassis/odom` | `nav_msgs/Odometry` | Robot position and velocity (ground truth from sim) |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree -- spatial relationships between all frames |
| `/clock` | `rosgraph_msgs/Clock` | Simulation time |

!!! note "Odometry is ground truth"
    In simulation, `/chassis/odom` is perfect -- no drift, no noise. On a real robot, odometry accumulates error over time. This is why SLAM (like cuVSLAM) exists: to correct drift using visual landmarks.

---

## Control

| Topic | Type | Description |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (`linear.x` = forward/back, `angular.z` = turn) |

---

## Added by Container

These topics are created by nodes in the Isaac ROS container, not by Isaac Sim itself.

| Topic | Type | Description |
|---|---|---|
| `/front_stereo_camera/left/compressed_video` | `foxglove_msgs/CompressedVideo` | H.264 NVENC compressed stream (from republisher) |
| `/joy` | `sensor_msgs/Joy` | Joystick input (from Foxglove joystick extension) |

---

## Unused Topics

The image_transport republisher auto-advertises these topics but nothing publishes to them:

`/out`, `/out/compressed`, `/out/compressedDepth`, `/out/theora`, `/out/zstd`

These are harmless artifacts of the image_transport plugin system.

---

## TF Tree

The Nova Carter robot has a large sensor suite. The transform tree shows how every frame relates to `base_link`:

```
odom                              <- world-fixed reference frame
 +-- base_link                    <- robot center (ground plane)
      |-- nova_carter             <- robot body
      |    |-- wheel_left
      |    |-- wheel_right
      |    +-- caster_frame_base
      |         |-- caster_swivel_left -> caster_wheel_left
      |         +-- caster_swivel_right -> caster_wheel_right
      |-- chassis_imu
      |-- front_3d_lidar
      |-- front/back_2d_lidar
      |-- front_stereo_camera     (left_optical, right_optical, imu)
      |-- left_stereo_camera      (left_optical, right_optical, imu)
      |-- right_stereo_camera     (left_optical, right_optical, imu)
      |-- rear_stereo_camera      (left_optical, right_optical, imu)
      |-- front/left/right/back_fisheye_camera_optical
      |-- front/left/right/back_hawk (stereo pairs)
      +-- mount
```

Each transform is a 4x4 homogeneous matrix (rotation + translation). ROS uses the TF tree to convert points between frames -- e.g., transforming a lidar point from the `front_3d_lidar` frame to `odom` (world coordinates).

---

## Useful Commands

```bash
# List all topics
ros2 topic list

# See live data on a topic
ros2 topic echo /front_stereo_camera/left/camera_info

# Check topic type and publisher/subscriber counts
ros2 topic info /cmd_vel --verbose

# Generate the full TF tree as a PDF
ros2 run tf2_tools view_frames
```
