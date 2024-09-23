import os
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_param_builder import load_yaml, load_xacro
from launch import LaunchContext
from pathlib import Path
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

ctx = LaunchContext()
package_dir = FindPackageShare('mtc')
dofbot_pkg = FindPackageShare('dofbot')
srdf_path = PathJoinSubstitution([dofbot_pkg, 'urdf', 'robot.srdf']).perform(ctx)
urdf_path = PathJoinSubstitution([dofbot_pkg, 'urdf', 'robot.urdf']).perform(ctx)
with open(srdf_path, 'r') as file:
    semantic_content = file.read()

print("urdf_path: ", urdf_path)

kin = load_yaml(Path(PathJoinSubstitution([dofbot_pkg, 'config', 'kinematics.yaml']).perform(ctx)))

config_path = PathJoinSubstitution([package_dir, 'config']).perform(ctx)
controller_path = Path(os.path.join(config_path, "controllers.yaml"))
planner_path = Path(os.path.join(config_path, "pilz_industrial_motion_planner.yaml"))

def generate_launch_description():
    # planning_context
    moveit_config = (
        MoveItConfigsBuilder("dofbot")
        .robot_description(urdf_path)
        .trajectory_execution(controller_path)
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

     # Set planner ID parameter
    planner_parameters = {
        "planning_pipelines": (planner_path).as_posix(),
        "planning_pipeline_configs": {
            "pilz_industrial_motion_planner": {
                "default_planner_id": "PTP"
            }
        }
    }
    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
          #  planner_parameters,
            move_group_capabilities,
        ],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("mtc") + "/config/dofbot.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(config_path, "dofbot.rviz")],
        parameters=[
            {"robot_description": Command(['xacro ', urdf_path])},
            {"robot_description_semantic": semantic_content},
            {"robot_description_kinematics": kin},
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": Command(['xacro ', urdf_path])},
        ],
    )

     
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_path.absolute().as_posix()],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )


    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller", "-c", "/controller_manager", "-p", controller_path.absolute().as_posix(), "--ros-args", "--log-level", ["debug"]],
            output="screen"
        ),


         Node(
            package="controller_manager",
            executable="spawner",
            arguments=["hand_controller", "-c", "/controller_manager", "-p", controller_path.absolute().as_posix()],
            output="screen"
        ),
            ros2_control_node,
        ]
    )
