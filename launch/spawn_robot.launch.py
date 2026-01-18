import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 1. Get the path to your XACRO file
    pkg_share = FindPackageShare("eng_description").find("eng_description")

    # Define the path to xacro file
    xacro_file_path = os.path.join(pkg_share, "urdf", "eng.xacro")

    # Check for file existence
    if not os.path.exists(xacro_file_path):
        raise FileNotFoundError(f"Xacro file not found at: {xacro_file_path}")

    # 2. Process Xacro to get robot description
    # Use Command to execute xacro and get URDF content
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_file_path,
            " use_sim:=true",  # Common xacro parameter for simulation
        ]
    )

    # 3. Launch Ignition Gazebo with empty world
    # Using 'ign gazebo' command for Ignition Gazebo
    gazebo_cmd = ExecuteProcess(
        cmd=["ign", "gazebo", "empty.sdf", "-v", "4"], output="screen", shell=False
    )

    # 4. Node to publish robot's transforms (using parameter instead of argument)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content, "use_sim_time": True}
        ],
    )

    # 5. Node to spawn the URDF model into Ignition Gazebo
    # Using topic-based spawning (recommended for URDF)
    spawn_entity = Node(
        package="ros_ign_gazebo",
        executable="create",
        arguments=[
            "-topic",
            "/robot_description",  # Read from topic instead of file
            "-name",
            "eng_robot",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.1",
            "-Y",
            "0.0",
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # 6. Delay the spawn to ensure Gazebo and robot_state_publisher are ready
    delayed_spawn = TimerAction(
        period=1.0,  # Wait 5 seconds
        actions=[spawn_entity],
    )

    # 7. Alternative: Use event handlers for proper sequencing
    spawn_after_gazebo = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gazebo_cmd, on_start=[robot_state_publisher]
        )
    )

    spawn_after_rsp = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher, on_start=[delayed_spawn]
        )
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
    )

    base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["base_controller"],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/eng/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/eng/cmd_vel_small@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/eng/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/model/eng/joint/joint_cronstein/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
            '/model/eng/joint/joint_first_part/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
            '/model/eng/joint/joint_second_part/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
            '/model/eng/joint/joint_head/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
        ],
        output='screen'
    )

    load_rsp = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gazebo_cmd,
            on_start=[robot_state_publisher]
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
                arm_controller_spawner,
                base_controller_spawner,
                bridge
            ]
        )
    )

    return LaunchDescription([
        gazebo_cmd,
        load_rsp,
        load_robot,
        load_controllers
    ])

