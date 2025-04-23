import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions      import Command
from launch_ros.actions       import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('dofbot_description')
    xacro_file = os.path.join(pkg, 'urdf', 'dofbot.urdf.xacro')

    # Expand the xacro to a URDF string
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' prefix:=', ''
    ])

    # 1) Launch Ignition Fortress
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': ''}.items()
    )

    # 2) Publish robot_description
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description_content
        }]
    )

    # 3) Spawn entity
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'dofbot', '-topic', 'robot_description'],
        output='screen'
    )

    # (Optional) GUI sliders
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    return LaunchDescription([
        gz,
        rsp,
        spawn,
        jsp_gui,
    ])
