from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_param_builder import load_xacro, load_yaml
from pathlib import Path
import os
from launch import LaunchContext
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

ctx = LaunchContext()
package_dir = FindPackageShare('dofbot')
srdf_path = PathJoinSubstitution([package_dir, 'urdf', 'robot.srdf']).perform(ctx)
urdf_path = PathJoinSubstitution([package_dir, 'urdf', 'robot.urdf']).perform(ctx)
with open(srdf_path, 'r') as f:
    semantic_content = f.read()
kin = load_yaml(Path(PathJoinSubstitution([package_dir, 'config', 'kinematics.yaml']).perform(ctx)))
controller_list = load_yaml(Path(PathJoinSubstitution([package_dir, 'config', 'simple_moveit_controllers.yaml']).perform(ctx)))

config_path = PathJoinSubstitution([package_dir, 'config']).perform(ctx)
should_publish = True
controller_path = Path(os.path.join(config_path, 'controllers.yaml'))

def generate_launch_description():
    pi_controller = Node(
          package="controller_manager",
          executable="ros2_control_node",
          parameters=[controller_path.absolute().as_posix()],
          remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
          ],
          output="screen"
        )
    nodes = [
         Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        ), 
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}]
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller", "-c", "/controller_manager", "-p", controller_path.absolute().as_posix()],
            output="screen"
        ),
#        Node(
#            package="controller_manager",
#            executable="spawner",
#            arguments=["hand_controller", "-c", "/controller_manager", "-p", controller_path.absolute().as_posix()],
#            output="screen"
#        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=[
                 controller_list,
                {"publish_robot_description_semantic": True},
                {"allow_trajectory_execution": True},
                {"publish_planning_scene": should_publish},
                {"publish_geometry_updates": should_publish},
                {"publish_state_updates": should_publish},
                {"publish_transforms_updates": should_publish},
                {"monitor_dynamics": False},
                {"use_sim_time": True},
                {'robot_description': Command(['xacro ', urdf_path])},
                {'robot_description_semantic':semantic_content},
                {'robot_description_kinematics': kin},
                {'move_group_capabilities': { "capabilities": "move_group/ExecuteTaskSolutionCapability" }},
                {'robot_description_planning': {'cartesian_limits': {'max_trans_vel': 1.0, 'max_trans_acc': 2.25, 'max_trans_dec': -5.0, 'max_rot_vel': 1.57}, 'joint_limits': { 'joint1': {'has_velocity_limits': True, 'max_velocity': 2.175, 'has_acceleration_limits': True, 'max_acceleration': 3.75, 'has_jerk_limits': False}, 'joint2': {'has_velocity_limits': True, 'max_velocity': 2.175, 'has_acceleration_limits': True, 'max_acceleration': 1.875, 'has_jerk_limits': False}, 'joint3': {'has_velocity_limits': True, 'max_velocity': 2.175, 'has_acceleration_limits': True, 'max_acceleration': 2.5, 'has_jerk_limits': False}, 'joint4': {'has_velocity_limits': True, 'max_velocity': 2.175, 'has_acceleration_limits': True, 'max_acceleration': 3.125, 'has_jerk_limits': False}, 'joint5': {'has_velocity_limits': True, 'max_velocity': 2.61, 'has_acceleration_limits': True, 'max_acceleration': 3.75, 'has_jerk_limits': False}, 'gripper': {'has_velocity_limits': True, 'max_velocity': 2.61, 'has_acceleration_limits': True, 'max_acceleration': 3.75, 'has_jerk_limits': False}, 'gripper_right': {'has_velocity_limits': True, 'max_velocity': 2.61, 'has_acceleration_limits': True, 'max_acceleration': 3.75, 'has_jerk_limits': False}} }},
                {'planning_pipelines': ['ompl', 'pilz_industrial_motion_planner', 'stomp', 'chomp'], 'default_planning_pipeline': 'ompl', 'ompl': {'planning_plugins': ['ompl_interface/OMPLPlanner'], 'request_adapters': ['default_planning_request_adapters/ResolveConstraintFrames', 'default_planning_request_adapters/ValidateWorkspaceBounds', 'default_planning_request_adapters/CheckStartStateBounds', 'default_planning_request_adapters/CheckStartStateCollision'], 'response_adapters': ['default_planning_response_adapters/AddTimeOptimalParameterization', 'default_planning_response_adapters/ValidateSolution', 'default_planning_response_adapters/DisplayMotionPath'], 'planner_configs': {'APSConfigDefault': {'type': 'geometric::AnytimePathShortening', 'shortcut': 1, 'hybridize': 1, 'max_hybrid_paths': 36, 'num_planners': 8, 'planners': 'RRTConnect,RRTConnect,RRTConnect,RRTConnect,RRTConnect,RRTConnect,RRTConnect,RRTConnect'}, 'SBLkConfigDefault': {'type': 'geometric::SBL', 'range': 0.0}, 'ESTkConfigDefault': {'type': 'geometric::EST', 'range': 0.0, 'goal_bias': 0.05}, 'LBKPIECEkConfigDefault': {'type': 'geometric::LBKPIECE', 'range': 0.0, 'border_fraction': 0.9, 'min_valid_path_fraction': 0.5}, 'BKPIECEkConfigDefault': {'type': 'geometric::BKPIECE', 'range': 0.0, 'border_fraction': 0.9, 'failed_expansion_score_factor': 0.5, 'min_valid_path_fraction': 0.5}, 'KPIECEkConfigDefault': {'type': 'geometric::KPIECE', 'range': 0.0, 'goal_bias': 0.05, 'border_fraction': 0.9, 'failed_expansion_score_factor': 0.5, 'min_valid_path_fraction': 0.5}, 'RRTkConfigDefault': {'type': 'geometric::RRT', 'range': 0.0, 'goal_bias': 0.05}, 'RRTConnectkConfigDefault': {'type': 'geometric::RRTConnect', 'range': 0.0}, 'RRTstarkConfigDefault': {'type': 'geometric::RRTstar', 'range': 0.0, 'goal_bias': 0.05, 'delay_collision_checking': 1}, 'TRRTkConfigDefault': {'type': 'geometric::TRRT', 'range': 0.0, 'goal_bias': 0.05, 'max_states_failed': 10, 'temp_change_factor': 2.0, 'min_temperature': '10e-10', 'init_temperature': '10e-6', 'frountier_threshold': 0.0, 'frountierNodeRatio': 0.1, 'k_constant': 0.0}, 'PRMkConfigDefault': {'type': 'geometric::PRM', 'max_nearest_neighbors': 10}, 'PRMstarkConfigDefault': {'type': 'geometric::PRMstar'}, 'FMTkConfigDefault': {'type': 'geometric::FMT', 'num_samples': 1000, 'radius_multiplier': 1.1, 'nearest_k': 1, 'cache_cc': 1, 'heuristics': 0, 'extended_fmt': 1}, 'BFMTkConfigDefault': {'type': 'geometric::BFMT', 'num_samples': 1000, 'radius_multiplier': 1.0, 'nearest_k': 1, 'balanced': 0, 'optimality': 1, 'heuristics': 1, 'cache_cc': 1, 'extended_fmt': 1}, 'PDSTkConfigDefault': {'type': 'geometric::PDST'}, 'STRIDEkConfigDefault': {'type': 'geometric::STRIDE', 'range': 0.0, 'goal_bias': 0.05, 'use_projected_distance': 0, 'degree': 16, 'max_degree': 18, 'min_degree': 12, 'max_pts_per_leaf': 6, 'estimated_dimension': 0.0, 'min_valid_path_fraction': 0.2}, 'BiTRRTkConfigDefault': {'type': 'geometric::BiTRRT', 'range': 0.0, 'temp_change_factor': 0.1, 'init_temperature': 100, 'frountier_threshold': 0.0, 'frountier_node_ratio': 0.1, 'cost_threshold': '1e300'}, 'LBTRRTkConfigDefault': {'type': 'geometric::LBTRRT', 'range': 0.0, 'goal_bias': 0.05, 'epsilon': 0.4}, 'BiESTkConfigDefault': {'type': 'geometric::BiEST', 'range': 0.0}, 'ProjESTkConfigDefault': {'type': 'geometric::ProjEST', 'range': 0.0, 'goal_bias': 0.05}, 'LazyPRMkConfigDefault': {'type': 'geometric::LazyPRM', 'range': 0.0}, 'LazyPRMstarkConfigDefault': {'type': 'geometric::LazyPRMstar'}, 'SPARSkConfigDefault': {'type': 'geometric::SPARS', 'stretch_factor': 3.0, 'sparse_delta_fraction': 0.25, 'dense_delta_fraction': 0.001, 'max_failures': 1000}, 'SPARStwokConfigDefault': {'type': 'geometric::SPARStwo', 'stretch_factor': 3.0, 'sparse_delta_fraction': 0.25, 'dense_delta_fraction': 0.001, 'max_failures': 5000}, 'TrajOptDefault': {'type': 'geometric::TrajOpt'}}, 'arm': {'planner_configs': ['APSConfigDefault', 'SBLkConfigDefault', 'ESTkConfigDefault', 'LBKPIECEkConfigDefault', 'BKPIECEkConfigDefault', 'KPIECEkConfigDefault', 'RRTkConfigDefault', 'RRTConnectkConfigDefault', 'RRTstarkConfigDefault', 'TRRTkConfigDefault', 'PRMkConfigDefault', 'PRMstarkConfigDefault', 'FMTkConfigDefault', 'BFMTkConfigDefault', 'PDSTkConfigDefault', 'STRIDEkConfigDefault', 'BiTRRTkConfigDefault', 'LBTRRTkConfigDefault', 'BiESTkConfigDefault', 'ProjESTkConfigDefault', 'LazyPRMkConfigDefault', 'LazyPRMstarkConfigDefault', 'SPARSkConfigDefault', 'SPARStwokConfigDefault', 'TrajOptDefault']}, 'arm_hand': {'planner_configs': ['APSConfigDefault', 'SBLkConfigDefault', 'ESTkConfigDefault', 'LBKPIECEkConfigDefault', 'BKPIECEkConfigDefault', 'KPIECEkConfigDefault', 'RRTkConfigDefault', 'RRTConnectkConfigDefault', 'RRTstarkConfigDefault', 'TRRTkConfigDefault', 'PRMkConfigDefault', 'PRMstarkConfigDefault', 'FMTkConfigDefault', 'BFMTkConfigDefault', 'PDSTkConfigDefault', 'STRIDEkConfigDefault', 'BiTRRTkConfigDefault', 'LBTRRTkConfigDefault', 'BiESTkConfigDefault', 'ProjESTkConfigDefault', 'LazyPRMkConfigDefault', 'LazyPRMstarkConfigDefault', 'SPARSkConfigDefault', 'SPARStwokConfigDefault', 'TrajOptDefault']}, 'hand': {'planner_configs': ['APSConfigDefault', 'SBLkConfigDefault', 'ESTkConfigDefault', 'LBKPIECEkConfigDefault', 'BKPIECEkConfigDefault', 'KPIECEkConfigDefault', 'RRTkConfigDefault', 'RRTConnectkConfigDefault', 'RRTstarkConfigDefault', 'TRRTkConfigDefault', 'PRMkConfigDefault', 'PRMstarkConfigDefault', 'FMTkConfigDefault', 'BFMTkConfigDefault', 'PDSTkConfigDefault', 'STRIDEkConfigDefault', 'BiTRRTkConfigDefault', 'LBTRRTkConfigDefault', 'BiESTkConfigDefault', 'ProjESTkConfigDefault', 'LazyPRMkConfigDefault', 'LazyPRMstarkConfigDefault', 'SPARSkConfigDefault', 'SPARStwokConfigDefault', 'TrajOptDefault']}}, 'pilz_industrial_motion_planner': {'planning_plugins': ['pilz_industrial_motion_planner/CommandPlanner'], 'request_adapters': ['default_planning_request_adapters/ValidateWorkspaceBounds', 'default_planning_request_adapters/CheckStartStateBounds', 'default_planning_request_adapters/CheckStartStateCollision'], 'default_planner_config': 'PTP', 'capabilities': 'pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService'}, 'stomp': {'planning_plugins': ['stomp_moveit/StompPlanner'], 'request_adapters': ['default_planning_request_adapters/ResolveConstraintFrames', 'default_planning_request_adapters/ValidateWorkspaceBounds', 'default_planning_request_adapters/CheckStartStateBounds', 'default_planning_request_adapters/CheckStartStateCollision'], 'response_adapters': ['default_planning_response_adapters/AddTimeOptimalParameterization', 'default_planning_response_adapters/ValidateSolution', 'default_planning_response_adapters/DisplayMotionPath'], 'stomp_moveit': {'num_timesteps': 60, 'num_iterations': 40, 'num_iterations_after_valid': 0, 'num_rollouts': 30, 'max_rollouts': 30, 'exponentiated_cost_sensitivity': 0.5, 'control_cost_weight': 0.1, 'delta_t': 0.1}}, 'chomp': {'planning_plugins': ['chomp_interface/CHOMPPlanner'], 'request_adapters': ['default_planning_request_adapters/ResolveConstraintFrames', 'default_planning_request_adapters/ValidateWorkspaceBounds', 'default_planning_request_adapters/CheckStartStateBounds', 'default_planning_request_adapters/CheckStartStateCollision'], 'response_adapters': ['default_planning_response_adapters/AddTimeOptimalParameterization'], 'planning_time_limit': 10.0, 'max_iterations': 200, 'max_iterations_after_collision_free': 5, 'smoothness_cost_weight': 0.1, 'obstacle_cost_weight': 1.0, 'learning_rate': 0.01, 'animate_path': True, 'add_randomness': False, 'smoothness_cost_velocity': 0.0, 'smoothness_cost_acceleration': 1.0, 'smoothness_cost_jerk': 0.0, 'hmc_discretization': 0.01, 'hmc_stochasticity': 0.01, 'hmc_annealing_factor': 0.99, 'use_hamiltonian_monte_carlo': False, 'ridge_factor': 0.0, 'use_pseudo_inverse': False, 'pseudo_inverse_ridge_factor': '1e-4', 'animate_endeffector': False, 'animate_endeffector_segment': 'rightfinger', 'joint_update_limit': 0.1, 'collision_clearance': 0.2, 'collision_threshold': 0.07, 'random_jump_amount': 1.0, 'use_stochastic_descent': True, 'enable_failure_recovery': False, 'max_recovery_attempts': 5, 'trajectory_initialization_method': 'quintic-spline'}}
            ]),
        pi_controller
    ]

    return LaunchDescription(nodes) 


