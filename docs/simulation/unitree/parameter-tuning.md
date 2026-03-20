# Parameter Tuning Guide

Every tunable parameter for G1 locomotion training. Two files control everything.

---

## File 1: Environment Config

**Path:** `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/velocity_env_cfg.py`

This file defines the robot, terrain, observations, rewards, and termination conditions.

### Reward Weights

These are the most impactful parameters. Positive weights encourage behavior, negative weights penalize it.

**Task rewards (what you want the robot to do):**

| Term | Weight | What it does |
|------|--------|-------------|
| `track_lin_vel_xy` | **+1.0** | Follow commanded forward/lateral speed |
| `track_ang_vel_z` | **+0.5** | Follow commanded yaw (turning) rate |
| `alive` | **+0.15** | Survive the episode without falling |
| `gait` | **+0.5** | Maintain alternating left-right stepping pattern (period: 0.8s) |
| `feet_clearance` | **+1.0** | Lift feet to 10cm above ground (clean stepping) |

**Penalties (what you want the robot to avoid):**

| Term | Weight | What it does |
|------|--------|-------------|
| `base_height` | **-10.0** | Deviating from 0.78m standing height |
| `flat_orientation_l2` | **-5.0** | Tilting (stay upright) |
| `dof_pos_limits` | **-5.0** | Joints approaching their limits |
| `base_linear_velocity` | **-2.0** | Vertical bouncing (z-axis velocity) |
| `joint_deviation_waists` | **-1.0** | Waist joints deviating from default |
| `joint_deviation_legs` | **-1.0** | Hip roll/yaw deviating from default |
| `undesired_contacts` | **-1.0** | Non-foot body parts touching the ground |
| `feet_slide` | **-0.2** | Feet sliding on ground while in contact |
| `joint_deviation_arms` | **-0.1** | Arm joints deviating from default pose |
| `base_angular_velocity` | **-0.05** | Roll/pitch angular velocity |
| `action_rate` | **-0.05** | Rapid changes in joint targets (jitter) |
| `joint_vel` | **-0.001** | Joint velocities (energy efficiency) |
| `joint_acc` | **-2.5e-7** | Joint accelerations (smoothness) |
| `energy` | **-2e-5** | Total energy consumption |

**Tuning tips:**

- Increase `track_lin_vel_xy` to make the robot prioritize speed over stability
- Increase `flat_orientation_l2` to make it more conservative about tilting
- Increase `alive` to make survival the top priority (useful early in a new config)
- Increase `gait` to enforce a more rhythmic stepping pattern
- Decrease `joint_deviation_arms` (closer to 0) to let arms move more freely

### Velocity Command Ranges

Controls what velocities are sampled during training:

```python
# Initial training range (starts conservative, widens via curriculum)
lin_vel_x=(-0.1, 0.1)    # forward/backward (m/s)
lin_vel_y=(-0.1, 0.1)    # lateral (m/s)
ang_vel_z=(-0.1, 0.1)    # yaw rate (rad/s)

# Maximum range (curriculum expands to this)
lin_vel_x=(-0.5, 1.0)    # max: 1 m/s forward, 0.5 m/s backward
lin_vel_y=(-0.3, 0.3)    # max: 0.3 m/s lateral
ang_vel_z=(-0.2, 0.2)    # max: 0.2 rad/s turning
```

Change `limit_ranges` to train for faster walking or sharper turns.

### Domain Randomization (Events)

Randomizations applied during training to improve robustness:

| Event | When | What it does |
|-------|------|-------------|
| `physics_material` | Startup | Randomize ground friction (0.3 to 1.0) |
| `add_base_mass` | Startup | Add -1 to +3 kg to the torso (simulates payloads) |
| `push_robot` | Every 5s | Push the robot with random velocity (up to 0.5 m/s) |
| `reset_base` | Reset | Randomize starting position and yaw |
| `reset_robot_joints` | Reset | Randomize initial joint positions |

**Tuning tips:**

- Increase `push_robot` velocity range to train a more stable robot
- Widen `add_base_mass` range if the robot will carry objects
- Increase `static_friction_range` minimum to avoid training on very slippery surfaces

### Termination Conditions

When an episode ends:

| Condition | Threshold | What it means |
|-----------|-----------|--------------|
| `time_out` | 20s (1000 steps) | Episode length limit |
| `base_height` | Below 0.2m | Robot has fallen down |
| `bad_orientation` | Tilt > 0.8 rad (~46 degrees) | Robot is tipping over |

Lower `bad_orientation` limit to terminate earlier when tilting (stricter). Raise it to let the robot recover from bigger tilts.

### Terrain

Currently set to flat terrain:

```python
sub_terrains={
    "flat": terrain_gen.MeshPlaneTerrainCfg(proportion=0.5),
}
```

Can be changed to include slopes, stairs, or rough terrain for more challenging training.

### Simulation Parameters

```python
decimation = 4          # policy runs every 4 physics steps
episode_length_s = 20.0 # 20-second episodes
sim.dt = 0.005          # physics timestep (200 Hz)
# Policy frequency = 200 / 4 = 50 Hz
```

### Observations

What the policy sees (in order):

| Term | Description | Noise |
|------|-------------|-------|
| `base_ang_vel` | Body angular velocity (scaled 0.2x) | +/- 0.2 |
| `projected_gravity` | Gravity direction in body frame | +/- 0.05 |
| `velocity_commands` | Commanded vx, vy, yaw_rate | None |
| `joint_pos_rel` | Joint positions relative to default | +/- 0.01 |
| `joint_vel_rel` | Joint velocities (scaled 0.05x) | +/- 1.5 |
| `last_action` | Previous policy output | None |

History length: **5 timesteps** (all observations stacked across 5 steps, creating the 480-dim input)

### Action Space

```python
JointPositionAction = mdp.JointPositionActionCfg(
    asset_name="robot",
    joint_names=[".*"],    # all 29 joints
    scale=0.25,            # output scaled by 0.25
    use_default_offset=True  # actions are offsets from default pose
)
```

`scale=0.25` means a policy output of 1.0 moves the joint 0.25 radians from default. Increase for more aggressive movements.

---

## File 2: Training Algorithm Config

**Path:** `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/agents/rsl_rl_ppo_cfg.py`

This file controls the neural network architecture and learning hyperparameters.

### Network Architecture

```python
actor_hidden_dims=[512, 256, 128]   # policy network layers
critic_hidden_dims=[512, 256, 128]  # value function layers
activation="elu"                     # activation function
init_noise_std=1.0                   # initial exploration noise
```

Larger networks learn more complex behaviors but train slower. The current 512-256-128 architecture is standard for locomotion.

### Learning Hyperparameters

| Parameter | Value | What it does |
|-----------|-------|-------------|
| `learning_rate` | 1e-3 | Step size for weight updates |
| `schedule` | "adaptive" | Adjusts LR based on KL divergence |
| `desired_kl` | 0.01 | Target KL divergence (controls LR adaptation) |
| `num_learning_epochs` | 5 | Passes through collected data per update |
| `num_mini_batches` | 4 | Data split into 4 mini-batches per epoch |
| `gamma` | 0.99 | Discount factor (how much future reward matters) |
| `lam` | 0.95 | GAE lambda (bias-variance tradeoff in advantage estimation) |
| `clip_param` | 0.2 | PPO clipping range (prevents large policy updates) |
| `entropy_coef` | 0.01 | Exploration bonus (higher = more random exploration) |
| `max_grad_norm` | 1.0 | Gradient clipping (prevents exploding gradients) |

### Training Loop

| Parameter | Value | What it does |
|-----------|-------|-------------|
| `num_steps_per_env` | 24 | Steps collected per env before updating |
| `max_iterations` | 50000 | Maximum training iterations |
| `save_interval` | 100 | Save checkpoint every 100 iterations |

**Tuning tips:**

- Lower `learning_rate` (e.g. 5e-4) for more stable but slower training
- Increase `entropy_coef` if the robot gets stuck in a local optimum (e.g. standing still)
- Increase `num_learning_epochs` to squeeze more learning from each batch of experience
- `gamma=0.99` means the robot cares about reward up to ~100 steps in the future

---

## Summary: What to Change Where

| I want to... | File | Parameter |
|--------------|------|-----------|
| Make the robot walk faster | velocity_env_cfg.py | Increase `limit_ranges.lin_vel_x` |
| Make the robot more stable | velocity_env_cfg.py | Increase `flat_orientation_l2` weight |
| Let arms swing more freely | velocity_env_cfg.py | Decrease `joint_deviation_arms` weight toward 0 |
| Train on rough terrain | velocity_env_cfg.py | Change `sub_terrains` to include slopes/stairs |
| Train longer episodes | velocity_env_cfg.py | Increase `episode_length_s` |
| Use a bigger neural network | rsl_rl_ppo_cfg.py | Increase `actor_hidden_dims` |
| Train more cautiously | rsl_rl_ppo_cfg.py | Decrease `learning_rate` |
| More exploration | rsl_rl_ppo_cfg.py | Increase `entropy_coef` |
| Save checkpoints more often | rsl_rl_ppo_cfg.py | Decrease `save_interval` |
