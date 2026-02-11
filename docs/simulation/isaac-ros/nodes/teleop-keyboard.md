# teleop_twist_keyboard

Keyboard-based robot control from a terminal. Publishes `Twist` directly to `/cmd_vel` — no Joy conversion needed.

```bash
# In a tmux session
docker exec -it isaac_ros_dev_container bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && \
   export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_no_shm.xml && \
   ros2 run teleop_twist_keyboard teleop_twist_keyboard"
```

```
   u    i    o        i = forward       k = stop
   j    k    l        , = backward      q/z = speed up/down
   m    ,    .        j/l = turn        w/x = turn speed up/down
```
