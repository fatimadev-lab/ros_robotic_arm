import os
import sys
import numpy as np
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from scripts.ik_solver import ik_solve, forward_kinematics


def test_ik_targets():
    targets = [
        [0.5, 0.0, 0.2],
        [0.2, 0.1, 0.25],
    ]
    for t in targets:
        q = ik_solve(t, n_restarts=6)
        pos, _, _ = forward_kinematics(q)
        assert np.linalg.norm(pos - np.array(t)) < 6e-2
