import os
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    args = []
    if (len(sys.argv) >= 5):
        i = 4
        while i < len(sys.argv):
            args.append(sys.argv[i])
            i = i + 1
            
    
    moveit_config = (
        MoveItConfigsBuilder(robot_name="tm5-900")
        .robot_description(
            file_path="config/tm5-900.urdf.xacro"
        )
        .robot_description_semantic(file_path="config/tm5-900.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"]
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_config.joint_limits,
            moveit_config.robot_description,
            moveit_config.planning_pipelines,
            {"use_sim_time": True},
            ],
        arguments=["--ros-args", "--log-level", "info"],
    )
    
    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("tm_moveit_cpp_demo"), "launch"
    )
    rviz_full_config = os.path.join(rviz_base, "run_moveit_cpp.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base']
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[moveit_config.robot_description]
    )
    
    ros2_controllers_path = os.path.join(
        get_package_share_directory("tm_moveit_cpp_demo"),
        "config",
        "controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    # joint driver
    tm_driver_node = Node(
        package='tm_driver',
        executable='tm_driver',
        #name='tm_driver',
        output='screen',
        arguments=args
    )

    return LaunchDescription([
                              tm_driver_node, 
                              static_tf, 
                              robot_state_publisher, 
                              rviz_node, 
                              move_group_node,
                              ros2_control_node,
                              ])
