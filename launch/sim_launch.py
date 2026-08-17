from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Simple launch to start robot_state_publisher with the URDF.
    pkg_share = ''
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open('urdf/6dof_arm.urdf').read()}]
    )
    return LaunchDescription([rsp])
