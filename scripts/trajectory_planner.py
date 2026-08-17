#!/usr/bin/env python3
"""
Trajectory planner: cubic-spline between start and goal joint vectors.
Samples trajectory, checks simple spherical obstacle collisions by sampling FK.
Publishes a `trajectory_msgs/JointTrajectory` to `/arm_controller/command` if ROS is present.
"""
import time
import numpy as np

from ik_solver import forward_kinematics, ik_solve
try:
    from scipy.interpolate import CubicSpline
except Exception:
    CubicSpline = None
try:
    import rclpy
    from rclpy.node import Node
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    ROS2_AVAILABLE = True
except Exception:
    ROS2_AVAILABLE = False

# Simple collision test: sample end-effector positions and ensure outside spheres
import os
import yaml
SCRIPT_DIR = os.path.dirname(__file__)
PARAMS_PATH = os.path.join(SCRIPT_DIR, '..', 'config', 'arm_params.yaml')
params = {}
try:
    with open(PARAMS_PATH, 'r') as f:
        params = yaml.safe_load(f)
except Exception:
    params = {}

OBSTACLES = params.get('obstacles', [])

def collides(q):
    pos, _ = forward_kinematics(q)
    for obs in OBSTACLES:
        c = np.array(obs['center'])
        r = float(obs['radius'])
        if np.linalg.norm(pos - c) <= r:
            return True
    return False

def plan_joint_spline(q_start, q_goal, duration=4.0, samples=100):
    q_start = np.array(q_start); q_goal = np.array(q_goal)
    if CubicSpline is None:
        # fallback: linear interpolation
        t = np.linspace(0, 1, samples)
        traj = np.outer(1 - t, q_start) + np.outer(t, q_goal)
    else:
        t_knots = [0.0, 1.0]
        cs = CubicSpline(t_knots, np.vstack([q_start, q_goal]), axis=0)
        t = np.linspace(0, 1, samples)
        traj = cs(t)
    # collision check
    for i, q in enumerate(traj):
        if collides(q):
            raise RuntimeError(f'Collision detected at sample {i}')
    return traj, np.linspace(0, duration, samples)

def publish_trajectory_ros2(traj, times):
    rclpy.init()
    node = rclpy.create_node('trajectory_commander')
    pub = node.create_publisher(JointTrajectory, '/arm_controller/command', 10)
    msg = JointTrajectory()
    msg.joint_names = [f'joint_{i+1}' for i in range(6)]
    for q, t in zip(traj, times):
        p = JointTrajectoryPoint()
        p.positions = q.tolist()
        # Duration expects sec/nanosec
        sec = int(t)
        nsec = int((t - sec) * 1e9)
        p.time_from_start = Duration(sec=sec, nanosec=nsec)
        msg.points.append(p)
    # publish once
    time.sleep(0.5)
    pub.publish(msg)
    node.get_logger().info('Trajectory published')
    rclpy.shutdown()


if __name__ == '__main__':
    # Simple demo: plan from current to target
    q_start = np.zeros(6)
    target = [0.4, 0.0, 0.2]
    q_goal = ik_solve(target, q0=q_start)
    traj, times = plan_joint_spline(q_start, q_goal)
    print('Planned', len(traj), 'samples; first/last joint sets:')
    print(traj[0])
    print(traj[-1])
    if ROS2_AVAILABLE:
        try:
            publish_trajectory_ros2(traj, times)
        except Exception as e:
            print('ROS2 publish failed:', e)
            print('Trajectory ready; sample count:', len(traj))
    else:
        print('ROS2 not available; trajectory ready; sample count:', len(traj))
