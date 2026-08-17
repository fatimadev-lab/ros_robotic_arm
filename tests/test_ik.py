#!/usr/bin/env python3
"""Simple smoke tests for IK solver.
Run: python tests/test_ik.py
"""
import os
import sys
# ensure repo root is on sys.path so `from scripts import ...` works
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from scripts.ik_solver import ik_solve, forward_kinematics

def run_tests():
    targets = [
        [0.5, 0.0, 0.2],
        [0.2, 0.1, 0.25],
        [0.3, -0.2, 0.15],
    ]
    for t in targets:
        try:
            q = ik_solve(t, n_restarts=6)
            pos, _ = forward_kinematics(q)
            print(f"Target: {t} -> q: {q} -> FK: {pos}")
        except Exception as e:
            print(f"IK failed for {t}: {e}")

if __name__ == '__main__':
    run_tests()
