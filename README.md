Robotic Arm Kinematics & Path Planning

Overview
- Minimal ROS package scaffold for a simulated 6-DOF robotic arm.
- Includes a URDF, Python IK (numerical) and trajectory planner (cubic spline).
- Simple collision checking using spherical obstacles sampled along trajectory.

Contents
- urdf/6dof_arm.urdf: simple chain URDF
- scripts/ik_solver.py: numerical IK + FK helper
- scripts/trajectory_planner.py: cubic-spline trajectory generator with collision checks
- launch/sim.launch: sample launch that can be adapted for Gazebo/RViz
- config/arm_params.yaml: link lengths, joint limits, obstacles

Requirements & Run (summary)
- Ubuntu + ROS (Noetic / Melodic) or ROS2 (adapt as needed)
- Python 3, numpy, scipy

Example (manual test):

1. Source your ROS setup: `source /opt/ros/noetic/setup.bash`
2. Start simulator (Gazebo) and spawn the URDF.
3. Run IK node: `rosrun ros_robotic_arm ik_solver.py`
4. Run planner: `rosrun ros_robotic_arm trajectory_planner.py`

See detailed notes in scripts' docstrings.

Quick start (updated)

1. Install Python 3.10+ and dependencies:

```powershell
pip install -r requirements.txt
```

2. Run a smoke test:

```powershell
python tests/test_ik.py
```

3. Run the CLI quick test:

```powershell
python scripts/ik_solver.py --target 0.5 0.0 0.2
```

Notes

- `scripts/ik_solver.py` prefers kinematics from `urdf/6dof_arm.urdf` if present; otherwise it falls back to DH parameters in `config/arm_params.yaml`.
- Use `ik_solve(..., target_quat=..., n_restarts=...)` to request orientation and multi-start restarts.
