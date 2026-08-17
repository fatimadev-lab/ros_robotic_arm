#!/usr/bin/env python3
"""
Numerical IK solver for a 6-DOF arm using scipy.optimize.
Provides: forward_kinematics(q), ik_solve(target_xyz)
Usage: can be imported or run as a standalone test node.
"""
import json
import math
import numpy as np
try:
    from scipy.optimize import minimize
except Exception:
    minimize = None
try:
    from scipy.optimize import least_squares
except Exception:
    least_squares = None
try:
    import rclpy
    ROS2_AVAILABLE = True
except Exception:
    ROS2_AVAILABLE = False

# Load params
import os
SCRIPT_DIR = os.path.dirname(__file__)
PARAMS_PATH = os.path.join(SCRIPT_DIR, '..', 'config', 'arm_params.yaml')

def load_params(path):
    import yaml
    with open(path, 'r') as f:
        return yaml.safe_load(f)

params = None
try:
    params = load_params(PARAMS_PATH)
except Exception:
    params = {'link_lengths':[0.1,0.4,0.3,0.05,0.05,0.02]}

LINKS = params['link_lengths']
JOINT_LIMITS = params.get('joint_limits', {})

# Attempt to parse URDF for kinematics. If parsing fails or URDF missing,
# we'll continue using DH-based FK as a fallback.
URDF_PATH = os.path.join(SCRIPT_DIR, '..', 'urdf', '6dof_arm.urdf')
URDF_AVAILABLE = False
JOINT_CHAIN = []  # list of joints along the main serial chain

def _parse_floats(s):
    if s is None:
        return None
    return [float(x) for x in s.strip().split()]

def _rpy_to_rot(rpy):
    # roll-pitch-yaw to rotation matrix (3x3)
    r, p, y = rpy
    cr = math.cos(r); sr = math.sin(r)
    cp = math.cos(p); sp = math.sin(p)
    cy = math.cos(y); sy = math.sin(y)
    Rr = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    Rp = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Ry = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    return Ry.dot(Rp).dot(Rr)

def _axis_angle_transform(axis, angle):
    # Returns 4x4 homogeneous transform for rotation about `axis` by `angle`.
    axis = np.array(axis, dtype=float)
    if np.linalg.norm(axis) == 0:
        return np.eye(4)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c = math.cos(angle); s = math.sin(angle)
    C = 1 - c
    R = np.array([
        [x*x*C + c,   x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s, y*y*C + c,   y*z*C - x*s],
        [z*x*C - y*s, z*y*C + x*s, z*z*C + c]])
    T = np.eye(4)
    T[:3,:3] = R
    return T

def _make_origin_transform(xyz, rpy):
    T = np.eye(4)
    if xyz is not None:
        T[:3,3] = xyz
    if rpy is not None:
        T[:3,:3] = _rpy_to_rot(rpy)
    return T

def parse_urdf_chain(urdf_path):
    """Parse a serial kinematic chain from a URDF file.
    Returns a list of joint dictionaries in chain order from root->leaf.
    """
    import xml.etree.ElementTree as ET
    if not os.path.exists(urdf_path):
        return []
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    # Collect joints and parent/child relationships
    joints = {}
    parent_set = set()
    child_set = set()
    for j in root.findall('joint'):
        name = j.get('name')
        jtype = j.get('type')
        parent = j.find('parent').get('link')
        child = j.find('child').get('link')
        origin = j.find('origin')
        xyz = _parse_floats(origin.get('xyz')) if origin is not None and origin.get('xyz') else [0.0,0.0,0.0]
        rpy = _parse_floats(origin.get('rpy')) if origin is not None and origin.get('rpy') else [0.0,0.0,0.0]
        axis_node = j.find('axis')
        axis = _parse_floats(axis_node.get('xyz')) if axis_node is not None and axis_node.get('xyz') else [0.0,0.0,1.0]
        joints[name] = {'name': name, 'type': jtype, 'parent': parent, 'child': child, 'origin_xyz': xyz, 'origin_rpy': rpy, 'axis': axis}
        parent_set.add(parent); child_set.add(child)
    # Find root link: a parent that is never a child
    roots = list(parent_set - child_set)
    if not roots:
        return []
    root_link = roots[0]
    # Build adjacency map from parent -> joint
    children_map = {}
    for j in joints.values():
        children_map.setdefault(j['parent'], []).append(j)
    # Walk the chain until leaf (stop if branching)
    chain = []
    cur_link = root_link
    while True:
        children = children_map.get(cur_link, [])
        if not children:
            break
        if len(children) > 1:
            # branching: pick first but warn
            child = children[0]
        else:
            child = children[0]
        chain.append(child)
        cur_link = child['child']
    return chain

# Initialize URDF chain if file present
try:
    JOINT_CHAIN = parse_urdf_chain(URDF_PATH)
    if JOINT_CHAIN:
        URDF_AVAILABLE = True
except Exception:
    URDF_AVAILABLE = False

# Validate and normalize LINKS
if not isinstance(LINKS, (list, tuple)) or len(LINKS) < 6:
    # ensure at least 6 entries
    LINKS = (LINKS if isinstance(LINKS, (list, tuple)) else list(LINKS))
    while len(LINKS) < 6:
        LINKS.append(0.0)

# Simple serial chain FK using planar transforms per-link (approximate)
def dh_transform(a, alpha, d, theta):
    ct = math.cos(theta); st = math.sin(theta)
    ca = math.cos(alpha); sa = math.sin(alpha)
    return np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0,0,0,1]
    ])

def forward_kinematics(q):
    # Return end-effector position (x,y,z) and full transform.
    # Prefer URDF-based FK if available.
    if URDF_AVAILABLE:
        if q is None or len(q) < len(JOINT_CHAIN):
            raise ValueError('q must be an iterable with %d joint values' % len(JOINT_CHAIN))
        T = np.eye(4)
        joint_transforms = []
        for i, j in enumerate(JOINT_CHAIN):
            origin_xyz = j.get('origin_xyz', [0.0,0.0,0.0])
            origin_rpy = j.get('origin_rpy', [0.0,0.0,0.0])
            T = T.dot(_make_origin_transform(origin_xyz, origin_rpy))
            if j['type'] == 'revolute' or j['type'] == 'continuous':
                T = T.dot(_axis_angle_transform(j['axis'], q[i]))
            joint_transforms.append(T.copy())
        pos = T[:3,3]
        return pos, T, joint_transforms
    # Fallback: use a made-up DH table that approximates the URDF chain.
    # Allow overriding DH parameters from params file under 'dh'
    default_a = [0, 0, 0, 0, 0, 0]
    default_alpha = [0, -math.pi/2, 0, -math.pi/2, math.pi/2, -math.pi/2]
    default_d = [0.1, 0, 0.4, 0.3, 0.05, 0.05]
    dh = params.get('dh', None)
    if dh and isinstance(dh, dict):
        a = dh.get('a', default_a)
        alpha = dh.get('alpha', default_alpha)
        d = dh.get('d', default_d)
    else:
        # Use link lengths as the per-link d offsets when provided
        d = list(LINKS[:6]) if LINKS else default_d
        a = default_a
        alpha = default_alpha
    # Validate q length
    if q is None or len(q) < 6:
        raise ValueError('q must be an iterable with 6 joint values')
    T = np.eye(4)
    joint_transforms = []
    for i in range(6):
        T = T.dot(dh_transform(a[i], alpha[i], d[i], q[i]))
        joint_transforms.append(T.copy())
    pos = T[:3,3]
    return pos, T, joint_transforms

def ik_cost(q, target):
    pos, _, _ = forward_kinematics(q)
    return np.linalg.norm(pos - target)


def in_collision(joint_transforms, obstacles=None, link_sample=3):
    """Simple collision checker: checks joint positions and sampled points along links.

    - `joint_transforms`: list of 4x4 transforms for each joint (as returned by `forward_kinematics`)
    - `obstacles`: list of dicts with `center` and `radius` keys
    - `link_sample`: number of samples between joints to check (>=1)
    """
    if not obstacles:
        return False
    # extract joint positions
    pts = [T[:3,3] for T in joint_transforms]
    # sample along links: between consecutive joint positions
    sampled = []
    for i in range(len(pts)-1):
        p0 = np.array(pts[i])
        p1 = np.array(pts[i+1])
        for s in range(1, link_sample+1):
            t = s / (link_sample+1)
            sampled.append(p0*(1-t) + p1*t)
    all_pts = pts + sampled
    for obs in obstacles:
        center = np.array(obs.get('center', [0,0,0]), dtype=float)
        radius = float(obs.get('radius', 0.0))
        for p in all_pts:
            if np.linalg.norm(p - center) <= radius:
                return True
    return False

def _quat_to_rot(quat):
    # quat as [x, y, z, w] or [w, x, y, z] (try to detect).
    q = np.array(quat, dtype=float)
    if q.size != 4:
        raise ValueError('Quaternion must have 4 elements')
    # Heuristic: if last element > 0.5 treat as w
    if abs(q[0]) > 1 or abs(q[1]) > 1 or abs(q[2]) > 1:
        # assume w first
        w, x, y, z = q
    else:
        # assume x,y,z,w
        x, y, z, w = q
    # normalize
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n == 0:
        return np.eye(3)
    w /= n; x /= n; y /= n; z /= n
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x + y*y)]])
    return R


def ik_solve(target_xyz, q0=None, target_quat=None, position_weight=1.0, orientation_weight=0.0, n_restarts=5, random_seed=None, avoid_collisions=False):
    """Solve IK for a target position and optional orientation.

    - `target_xyz`: 3-element position
    - `target_quat`: optional 4-element quaternion (x,y,z,w or w,x,y,z)
    - `orientation_weight`: weight for orientation residual (0 = ignore)
    - `n_restarts`: number of random restarts (includes q0)
    Returns best joint vector found.
    """
    target = np.array(target_xyz, dtype=float)
    if target.shape != (3,) and target.shape != (3,1):
        raise ValueError('target_xyz must be a 3-element iterable (x,y,z)')
    if q0 is None:
        q0 = np.zeros(max(6, len(JOINT_CHAIN) if URDF_AVAILABLE else 6))
    else:
        q0 = np.array(q0, dtype=float)
    if q0.size < 6:
        raise ValueError('q0 must have length >= 6')

    # Build bounds safely from JOINT_LIMITS if provided
    bounds = None
    lower = None; upper = None
    if JOINT_LIMITS and isinstance(JOINT_LIMITS, dict):
        lower = JOINT_LIMITS.get('lower')
        upper = JOINT_LIMITS.get('upper')
        if (isinstance(lower, (list, tuple)) and isinstance(upper, (list, tuple))
                and len(lower) >= 6 and len(upper) >= 6):
            bounds = list(zip(lower[:6], upper[:6]))
            lb = np.array([b[0] for b in bounds], dtype=float)
            ub = np.array([b[1] for b in bounds], dtype=float)

    # Prepare orientation target
    R_target = None
    if target_quat is not None:
        R_target = _quat_to_rot(target_quat)

    # residual function combining position and orientation (flattened)
    def residuals_all(x):
        pos, T, jt = forward_kinematics(x)
        r_pos = position_weight * (pos - target)
        if R_target is None or orientation_weight == 0.0:
            return r_pos
        R_curr = T[:3,:3]
        # use Frobenius difference as a simple orientation residual
        r_ori = orientation_weight * (R_curr - R_target).ravel()
        return np.concatenate((r_pos, r_ori))

    # Build initial guess set: q0, mid-limits, plus randoms
    rng = np.random.default_rng(random_seed)
    inits = [np.array(q0, dtype=float)]
    if bounds is not None:
        mid = 0.5 * (lb + ub)
        inits.append(mid)
        for _ in range(max(0, n_restarts-2)):
            inits.append(rng.uniform(lb, ub))
    else:
        for _ in range(max(0, n_restarts-1)):
            inits.append(q0 + rng.normal(scale=0.5, size=q0.shape))

    best_x = None
    best_cost = float('inf')
    # Try least_squares per-init if available, otherwise fallback to minimize
    for start in inits:
        try:
            if least_squares is not None:
                if bounds is not None:
                    res = least_squares(residuals_all, start, bounds=(lb, ub), xtol=1e-6, ftol=1e-6, max_nfev=1000)
                else:
                    res = least_squares(residuals_all, start, xtol=1e-6, ftol=1e-6, max_nfev=1000)
                if res.success:
                    # check collisions
                    _, _, jt = forward_kinematics(res.x)
                    if avoid_collisions and in_collision(jt, params.get('obstacles', [])):
                        # reject
                        cost = float('inf')
                    else:
                        cost = float(np.sum(res.fun**2))
                        if cost < best_cost:
                            best_cost = cost
                            best_x = res.x
                else:
                    # still evaluate residuals and collision
                    _, _, jt = forward_kinematics(res.x)
                    if avoid_collisions and in_collision(jt, params.get('obstacles', [])):
                        cost = float('inf')
                    else:
                        cost = float(np.sum(res.fun**2))
                        if cost < best_cost:
                            best_cost = cost
                            best_x = res.x
            else:
                # fallback minimize on squared cost
                f = lambda x: float(np.sum(residuals_all(x)**2))
                resm = minimize(f, start, bounds=bounds, method='SLSQP', options={'ftol':1e-6, 'maxiter':500})
                if resm.success:
                    _, _, jt = forward_kinematics(resm.x)
                    if avoid_collisions and in_collision(jt, params.get('obstacles', [])):
                        cost = float('inf')
                    else:
                        cost = float(resm.fun)
                        if cost < best_cost:
                            best_cost = cost
                            best_x = resm.x
        except Exception:
            continue

    if best_x is None:
        raise RuntimeError('IK failed to find a solution')
    return best_x

if __name__ == '__main__':
    # quick CLI test
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--target', nargs=3, type=float, default=[0.5, 0.0, 0.2])
    parser.add_argument('--target-quat', nargs=4, type=float, default=None, help='Optional target orientation quaternion (x y z w or w x y z)')
    parser.add_argument('--orientation-weight', type=float, default=0.0, help='Weight for orientation residual')
    parser.add_argument('--n-restarts', type=int, default=5, help='Number of random restarts for IK')
    parser.add_argument('--avoid-collisions', action='store_true', help='Reject solutions in collision with configured obstacles')
    args = parser.parse_args()
    q = ik_solve(args.target, target_quat=args.target_quat, orientation_weight=args.orientation_weight, n_restarts=args.n_restarts, avoid_collisions=args.avoid_collisions)
    print('IK result (radians):', q)
    pos, _ = forward_kinematics(q)
    print('FK pos:', pos)
