# TensorBoard — Training Monitoring

Monitor G1 locomotion training progress in real time.

## Launch

```bash
conda activate isaaclab
cd ~/GIT/unitree_rl_lab
tensorboard --logdir logs/rsl_rl/ --host 0.0.0.0
```

Open [http://workstation:6006](http://workstation:6006) in a browser. TensorBoard auto-refreshes as new data arrives during training.

---

## Key Metrics

| Metric | What it means | Good trend |
|--------|--------------|------------|
| **Mean reward** | Overall training progress — weighted sum of all reward terms | Upward |
| **Mean episode length** | How long robots stay alive before termination | Upward |
| **Episode_Termination/bad_orientation** | Percentage of robots falling over | Downward |
| **Episode_Termination/time_out** | Robots surviving the full episode length | Upward |
| **Episode_Reward/track_lin_vel_xy** | How well the robot follows velocity commands | Upward |
| **Episode_Reward/feet_clearance** | Foot lift quality — clean stepping gait | Upward |

**Mean reward** is the single most important metric. The termination and reward breakdowns explain *why* it's going up or down.

---

## Training Phases

G1 locomotion training progresses through distinct phases:

| Phase | Behavior | Typical iteration |
|-------|----------|-------------------|
| 1. Random flailing | Joints move randomly, robots fall immediately | 0–50 |
| 2. Standing | Robots learn to stay upright but don't move | 50–200 |
| 3. Balancing | Robots shift weight, may shuffle in place | 200–500 |
| 4. Stepping | Recognizable leg movements, clumsy walking | 500–1000 |
| 5. Walking | Smooth gait, follows velocity commands | 1000+ |

Iteration ranges are approximate — varies with reward tuning and random seed.

---

## When to Stop Training

Training doesn't need to converge fully to be useful. Stop when:

- **Mean reward plateaus** for 200+ iterations with no improvement
- **bad_orientation** drops below ~5% (robots rarely fall)
- **track_lin_vel_xy** is high and stable (robot follows commands reliably)
- **Mean episode length** is near the maximum episode limit

On an RTX 3090, a good policy typically emerges within 1000–2000 iterations (~30 min). Diminishing returns after that — export the `.onnx` and test in sim before investing more training time.
