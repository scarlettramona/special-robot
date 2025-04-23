from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import FindExecutable

def generate_launch_description():
    pkg_dofbot = get_package_share_directory("dofbot_description")

    # Use FindExecutable to locate 'xacro'
    xacro_command = PathJoinSubstitution([
        pkg_dofbot,
        "urdf",
        "dofbot.urdf.xacro"
    ])

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        xacro_command
    ])

    # Gazebo classic launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": True, "robot_description": robot_description_content}],
        output="screen"
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "dofbot",
            "-topic",  "robot_description",
            "-z",      "0.2"          # 20 cm above ground
        ],
        output="screen"
    )

    jsb_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen"
            )
        ]
    )

    ptc_spawner = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["position_trajectory_controller", "--controller-manager", "/controller_manager"],
                output="screen"
            )
        ]
    )

    jsp_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen"
    )

    return LaunchDescription([
        gazebo_launch,
        rsp_node,
        spawn_entity,
        jsb_spawner,
        ptc_spawner,
        jsp_gui
    ])
