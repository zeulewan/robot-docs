# Simulation

Isaac Sim and Isaac ROS — simulation, perception, and the ROS 2 bridge.

<div style="display: flex; gap: 1em; justify-content: center; flex-wrap: wrap; margin: 1em auto;">
  <div style="flex: 1; min-width: 300px; max-width: 48%;">
    <video controls autoplay muted loop style="width: 100%; border-radius: 8px;">
      <source src="../assets/demo.mp4" type="video/mp4">
    </video>
    <p style="text-align: center; opacity: 0.7; font-size: 0.85em;">Carter robot warehouse demo with Foxglove Studio</p>
  </div>
  <div style="flex: 1; min-width: 300px; max-width: 48%;">
    <video controls autoplay muted loop style="width: 100%; border-radius: 8px;">
      <source src="../assets/g1-walking.mp4" type="video/mp4">
    </video>
    <p style="text-align: center; opacity: 0.7; font-size: 0.85em;">Unitree G1 locomotion policy trained with RL in Isaac Lab</p>
  </div>
</div>

<div class="grid cards" markdown>

-   :material-cube-outline:{ .lg .middle } **Isaac Sim**

    ---

    Isaac Sim 5.1 and Isaac Lab setup, ROS 2 bridge, Foxglove visualization

    [:octicons-arrow-right-24: Setup](isaac-sim/setup.md) · [:octicons-arrow-right-24: ROS 2 Topics](isaac-sim/topics.md)

-   :material-docker:{ .lg .middle } **Isaac ROS**

    ---

    GPU-accelerated perception container: AprilTag, cuVSLAM, nvblox

    [:octicons-arrow-right-24: Container](isaac-ros/container.md) · [:octicons-arrow-right-24: Nodes](isaac-ros/nodes/) · [:octicons-arrow-right-24: Dockerfile](isaac-ros/dockerfile.md)

-   :material-robot:{ .lg .middle } **Unitree**

    ---

    Unitree RL Lab, sim-to-real stack, URDF files, and G1 locomotion training

    [:octicons-arrow-right-24: Overview](unitree/)

</div>
