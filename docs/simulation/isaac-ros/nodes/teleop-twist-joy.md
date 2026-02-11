# teleop_node (teleop_twist_joy)

Converts joystick input to robot velocity commands.

| | |
|---|---|
| **Entrypoint** | `70-teleop-twist-joy.sh` |
| **Subscribes to** | `/joy` (`sensor_msgs/Joy`) |
| **Publishes** | `/cmd_vel` (`geometry_msgs/Twist`) |

Axis mapping:

| Joy axis | Twist field | Control |
|---|---|---|
| Axis 0 (horizontal) | `angular.z` | Turn left/right (0.5 rad/s max) |
| Axis 1 (vertical) | `linear.x` | Forward/backward (0.5 m/s max) |

The deadman switch is disabled (`require_enable_button:=false`) so input goes straight to `/cmd_vel`. Used with the foxglove-joystick extension in Foxglove, publishing to `/joy`.

!!! warning "ROS2 Schema Fix Required"
    The [original extension](https://github.com/joshnewans/foxglove-joystick) uses `sensor_msgs/Joy` (ROS1 schema). ROS2 requires `sensor_msgs/msg/Joy`, so the foxglove bridge rejects it with "Unknown message definition." Use the fixed build from [MSSergeev's fork](https://github.com/MSSergeev/foxglove-joystick) which adds ROS2 auto-detection.

    **Download:** [foxglove-joystick-ros2-fixed.foxe](../../../assets/foxglove-joystick-ros2-fixed.foxe)

    Install by dragging the `.foxe` file onto the Foxglove Studio window. If you have the original extension installed, remove it first (Profile → Extensions → uninstall).
