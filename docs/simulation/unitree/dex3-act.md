# Dex3 ACT Manipulation

This work uses Hugging Face LeRobot's ACT policy with the Unitree G1 and Dex3
hands in Isaac Lab. ACT is an imitation-learning architecture: it predicts short
chunks of robot actions from recent observations instead of selecting one action
at a time.

## Verified Simulation Result

<video controls muted loop style="width: 100%; max-width: 960px; display: block; margin: 1em auto;">
  <source src="/robot-docs/assets/dex3-act-unified-policy-full-stack.mp4" type="video/mp4">
</video>

The verified rollout uses one unified ACT checkpoint with about **34.2 million
parameters**. It completes a stable three-block stack in Isaac Lab using head
stereo images and joint state. The validation excluded reference replay,
retargeting, object snapping or pose writes, grasp assistance, and wrist-camera
mounts.

This is a simulation result, not a real-robot deployment result. Earlier clips
with a visually misaligned tabletop and staged checkpoint switching are
superseded by this unified-policy rollout.

## Local Project State

| Item | Value |
|---|---|
| Repository | `/home/zeul/GIT/unitree_lerobot` |
| Local commit | `41c2805` |
| Training framework | Hugging Face LeRobot |
| Policy | ACT |
| Robot/end effector | Unitree G1 with Dex3 hands |
| Inputs | Head stereo images plus joint state |
| Output | Arm and hand action chunks |

The checkout has local changes in `unitree_lerobot/eval_robot/eval_g1.py`, the
LeRobot submodule, camera configuration, and scripts. Preserve those changes
before updating or cleaning the repository.

## Data Path

```text
demonstration episodes
  -> LeRobot dataset
  -> ACT training
  -> checkpoint
  -> Isaac Lab policy rollout
  -> physical G1 evaluation only after hardware and safety gates
```

For new demonstrations, the Quest path can supply hand and arm motion while
Isaac Lab supplies synchronized camera and joint observations. Dataset schema,
camera placement, action dimensions, normalization statistics, and timing must
match between training and inference.

## References

- [Unitree LeRobot](https://github.com/unitreerobotics/unitree_lerobot)
- [Hugging Face LeRobot](https://github.com/huggingface/lerobot)
- [ACT policy documentation](https://huggingface.co/docs/lerobot/act)
