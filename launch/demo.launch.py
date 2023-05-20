from ament_index_python.packages import (
    get_package_share_path,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(
            file_path=get_package_share_path("pybullet_ros2")
            / "config"
            / "panda.urdf.xacro",
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"],
        )
        .to_moveit_configs()
    )

    # Pybullet
    pybullet_node = Node(
        package="pybullet_ros2",
        executable="pybullet_ros2_node",
        parameters=[moveit_config.robot_description],
    )
    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("pybullet_ros2"),
                    "launch",
                    "moveit.rviz",
                ],
            ),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            get_package_share_path("moveit_resources_panda_moveit_config")
            / "config"
            / "ros2_controllers.yaml",
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
    )

    panda_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_hand_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription(
        [
            pybullet_node,
            # rviz_node,
            static_tf_node,
            robot_state_publisher,
            # move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            panda_arm_controller_spawner,
            panda_hand_controller_spawner,
        ],
    )
