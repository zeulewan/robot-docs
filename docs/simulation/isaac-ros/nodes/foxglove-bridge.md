# foxglove_bridge

WebSocket bridge that exposes all ROS 2 topics to Foxglove Studio.

| | |
|---|---|
| **Entrypoint** | `50-foxglove-bridge.sh` |
| **Port** | 8765 |
| **Connect** | `ws://workstation:8765` |

Foxglove Studio connects to this bridge and can subscribe to any ROS 2 topic, publish messages (like `/joy` from the joystick panel), and call services. It's the single point of contact between the browser and the ROS 2 system.
