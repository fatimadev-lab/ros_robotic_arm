from setuptools import setup
import os

package_name = 'ros_robotic_arm'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/6dof_arm.urdf']),
        ('share/' + package_name + '/config', ['config/arm_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/sim_launch.py'])
    ],
    install_requires=['setuptools'],
    scripts=[
        'scripts/ik_solver.py',
        'scripts/trajectory_planner.py'
    ],
    zip_safe=True,
)
