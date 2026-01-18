import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare("eng_description").find("eng_description")
    xacro_file_path = os.path.join(pkg_share, "urdf", "eng.xacro")
    if not os.path.exists(xacro_file_path):
        raise FileNotFoundError(f"Xacro file not found at: {xacro_file_path}")

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_file_path,
            " use_sim:=true",
        ]
    )

    gazebo_cmd = ExecuteProcess(
        cmd=["ign", "gazebo", "empty.sdf", "-v", "4", "-r"], 
        output="screen",
        shell=False
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_content,
            "use_sim_time": True
        }]
    )

    spawn_entity = Node(
        package="ros_ign_gazebo",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "eng_robot",
            "-x", "0",
            "-y", "0", 
            "-z", "0.1",
            "-Y", "0.0"
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["base_controller"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
    )

    flipper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["flipper_position_controller"],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': True}]
    )

    load_rsp_and_bridge = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gazebo_cmd,
            on_start=[robot_state_publisher, bridge]
        )
    )

    load_robot = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[TimerAction(period=2.0, actions=[spawn_entity])]
        )
    )

    load_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                joint_state_broadcaster_spawner,
                base_controller_spawner,
                arm_controller_spawner,
                flipper_controller_spawner,
            ]
        )
    )

    return LaunchDescription([
        gazebo_cmd,
        load_rsp_and_bridge,
        # rviz_node,
        load_robot,
        load_controllers
    ])