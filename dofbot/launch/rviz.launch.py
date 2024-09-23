from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_param_builder import load_xacro, load_yaml
from pathlib import Path
from launch import LaunchContext
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os

ctx = LaunchContext()
package_dir = FindPackageShare('dofbot')
controller_list = load_yaml(Path(PathJoinSubstitution([package_dir, 'config', 'simple_moveit_controllers.yaml']).perform(ctx)))

config_path = PathJoinSubstitution([package_dir, 'config']).perform(ctx)
should_publish = True
controller_path = Path(os.path.join(config_path, 'controllers.yaml'))


ctx = LaunchContext()
package_dir = FindPackageShare('dofbot')
srdf_path = PathJoinSubstitution([package_dir, 'urdf', 'robot.srdf']).perform(ctx)
urdf_path = PathJoinSubstitution([package_dir, 'urdf', 'robot.urdf']).perform(ctx)
with open(srdf_path, 'r') as f:
    semantic_content = f.read()
kin = load_yaml(Path(PathJoinSubstitution([package_dir, 'config', 'kinematics.yaml']).perform(ctx)))
config_path = PathJoinSubstitution([package_dir, 'config', 'dofbot.rviz']).perform(ctx)

package_dir = FindPackageShare(LaunchConfiguration('urdf_package'))
print(PathJoinSubstitution([package_dir, 'urdf', 'robot.srdf']))

def generate_launch_description():
    rvis = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', config_path],
            parameters=[
                {'robot_description': Command(['xacro ', urdf_path])},
                {'robot_description_semantic': semantic_content},
                {'robot_description_kinematics': kin},
                {'robot_description_planning': {'cartesian_limits': {'max_trans_vel': 1.0, 'max_trans_acc': 2.25, 'max_trans_dec': -5.0, 'max_rot_vel': 1.57}, 'joint_limits': { 'joint1': {'has_velocity_limits': True, 'max_velocity': 2.175, 'has_acceleration_limits': True, 'max_acceleration': 3.75, 'has_jerk_limits': False}, 'joint2': {'has_velocity_limits': True, 'max_velocity': 2.175, 'has_acceleration_limits': True, 'max_acceleration': 1.875, 'has_jerk_limits': False}, 'joint3': {'has_velocity_limits': True, 'max_velocity': 2.175, 'has_acceleration_limits': True, 'max_acceleration': 2.5, 'has_jerk_limits': False}, 'joint4': {'has_velocity_limits': True, 'max_velocity': 2.175, 'has_acceleration_limits': True, 'max_acceleration': 3.125, 'has_jerk_limits': False}, 'joint5': {'has_velocity_limits': True, 'max_velocity': 2.61, 'has_acceleration_limits': True, 'max_acceleration': 3.75, 'has_jerk_limits': False}} }},
                {'planning_pipelines': ['chomp'], 'default_planning_pipeline': 'chomp', 'ompl': {'planning_plugins': ['ompl_interface/OMPLPlanner'], 'request_adapters': ['default_planning_request_adapters/ResolveConstraintFrames', 'default_planning_request_adapters/ValidateWorkspaceBounds', 'default_planning_request_adapters/CheckStartStateBounds', 'default_planning_request_adapters/CheckStartStateCollision'], 'response_adapters': ['default_planning_response_adapters/AddTimeOptimalParameterization', 'default_planning_response_adapters/ValidateSolution', 'default_planning_response_adapters/DisplayMotionPath'], 'planner_configs': {'APSConfigDefault': {'type': 'geometric::AnytimePathShortening', 'shortcut': 1, 'hybridize': 1, 'max_hybrid_paths': 36, 'num_planners': 8, 'planners': 'RRTConnect,RRTConnect,RRTConnect,RRTConnect,RRTConnect,RRTConnect,RRTConnect,RRTConnect'}, 'SBLkConfigDefault': {'type': 'geometric::SBL', 'range': 0.0}, 'ESTkConfigDefault': {'type': 'geometric::EST', 'range': 0.0, 'goal_bias': 0.05}, 'LBKPIECEkConfigDefault': {'type': 'geometric::LBKPIECE', 'range': 0.0, 'border_fraction': 0.9, 'min_valid_path_fraction': 0.5}, 'BKPIECEkConfigDefault': {'type': 'geometric::BKPIECE', 'range': 0.0, 'border_fraction': 0.9, 'failed_expansion_score_factor': 0.5, 'min_valid_path_fraction': 0.5}, 'KPIECEkConfigDefault': {'type': 'geometric::KPIECE', 'range': 0.0, 'goal_bias': 0.05, 'border_fraction': 0.9, 'failed_expansion_score_factor': 0.5, 'min_valid_path_fraction': 0.5}, 'RRTkConfigDefault': {'type': 'geometric::RRT', 'range': 0.0, 'goal_bias': 0.05}, 'RRTConnectkConfigDefault': {'type': 'geometric::RRTConnect', 'range': 0.0}, 'RRTstarkConfigDefault': {'type': 'geometric::RRTstar', 'range': 0.0, 'goal_bias': 0.05, 'delay_collision_checking': 1}, 'TRRTkConfigDefault': {'type': 'geometric::TRRT', 'range': 0.0, 'goal_bias': 0.05, 'max_states_failed': 10, 'temp_change_factor': 2.0, 'min_temperature': '10e-10', 'init_temperature': '10e-6', 'frountier_threshold': 0.0, 'frountierNodeRatio': 0.1, 'k_constant': 0.0}, 'PRMkConfigDefault': {'type': 'geometric::PRM', 'max_nearest_neighbors': 10}, 'PRMstarkConfigDefault': {'type': 'geometric::PRMstar'}, 'FMTkConfigDefault': {'type': 'geometric::FMT', 'num_samples': 1000, 'radius_multiplier': 1.1, 'nearest_k': 1, 'cache_cc': 1, 'heuristics': 0, 'extended_fmt': 1}, 'BFMTkConfigDefault': {'type': 'geometric::BFMT', 'num_samples': 1000, 'radius_multiplier': 1.0, 'nearest_k': 1, 'balanced': 0, 'optimality': 1, 'heuristics': 1, 'cache_cc': 1, 'extended_fmt': 1}, 'PDSTkConfigDefault': {'type': 'geometric::PDST'}, 'STRIDEkConfigDefault': {'type': 'geometric::STRIDE', 'range': 0.0, 'goal_bias': 0.05, 'use_projected_distance': 0, 'degree': 16, 'max_degree': 18, 'min_degree': 12, 'max_pts_per_leaf': 6, 'estimated_dimension': 0.0, 'min_valid_path_fraction': 0.2}, 'BiTRRTkConfigDefault': {'type': 'geometric::BiTRRT', 'range': 0.0, 'temp_change_factor': 0.1, 'init_temperature': 100, 'frountier_threshold': 0.0, 'frountier_node_ratio': 0.1, 'cost_threshold': '1e300'}, 'LBTRRTkConfigDefault': {'type': 'geometric::LBTRRT', 'range': 0.0, 'goal_bias': 0.05, 'epsilon': 0.4}, 'BiESTkConfigDefault': {'type': 'geometric::BiEST', 'range': 0.0}, 'ProjESTkConfigDefault': {'type': 'geometric::ProjEST', 'range': 0.0, 'goal_bias': 0.05}, 'LazyPRMkConfigDefault': {'type': 'geometric::LazyPRM', 'range': 0.0}, 'LazyPRMstarkConfigDefault': {'type': 'geometric::LazyPRMstar'}, 'SPARSkConfigDefault': {'type': 'geometric::SPARS', 'stretch_factor': 3.0, 'sparse_delta_fraction': 0.25, 'dense_delta_fraction': 0.001, 'max_failures': 1000}, 'SPARStwokConfigDefault': {'type': 'geometric::SPARStwo', 'stretch_factor': 3.0, 'sparse_delta_fraction': 0.25, 'dense_delta_fraction': 0.001, 'max_failures': 5000}, 'TrajOptDefault': {'type': 'geometric::TrajOpt'}}, 'arm': {'planner_configs': ['APSConfigDefault', 'SBLkConfigDefault', 'ESTkConfigDefault', 'LBKPIECEkConfigDefault', 'BKPIECEkConfigDefault', 'KPIECEkConfigDefault', 'RRTkConfigDefault', 'RRTConnectkConfigDefault', 'RRTstarkConfigDefault', 'TRRTkConfigDefault', 'PRMkConfigDefault', 'PRMstarkConfigDefault', 'FMTkConfigDefault', 'BFMTkConfigDefault', 'PDSTkConfigDefault', 'STRIDEkConfigDefault', 'BiTRRTkConfigDefault', 'LBTRRTkConfigDefault', 'BiESTkConfigDefault', 'ProjESTkConfigDefault', 'LazyPRMkConfigDefault', 'LazyPRMstarkConfigDefault', 'SPARSkConfigDefault', 'SPARStwokConfigDefault', 'TrajOptDefault']}, 'arm_hand': {'planner_configs': ['APSConfigDefault', 'SBLkConfigDefault', 'ESTkConfigDefault', 'LBKPIECEkConfigDefault', 'BKPIECEkConfigDefault', 'KPIECEkConfigDefault', 'RRTkConfigDefault', 'RRTConnectkConfigDefault', 'RRTstarkConfigDefault', 'TRRTkConfigDefault', 'PRMkConfigDefault', 'PRMstarkConfigDefault', 'FMTkConfigDefault', 'BFMTkConfigDefault', 'PDSTkConfigDefault', 'STRIDEkConfigDefault', 'BiTRRTkConfigDefault', 'LBTRRTkConfigDefault', 'BiESTkConfigDefault', 'ProjESTkConfigDefault', 'LazyPRMkConfigDefault', 'LazyPRMstarkConfigDefault', 'SPARSkConfigDefault', 'SPARStwokConfigDefault', 'TrajOptDefault']}, 'hand': {'planner_configs': ['APSConfigDefault', 'SBLkConfigDefault', 'ESTkConfigDefault', 'LBKPIECEkConfigDefault', 'BKPIECEkConfigDefault', 'KPIECEkConfigDefault', 'RRTkConfigDefault', 'RRTConnectkConfigDefault', 'RRTstarkConfigDefault', 'TRRTkConfigDefault', 'PRMkConfigDefault', 'PRMstarkConfigDefault', 'FMTkConfigDefault', 'BFMTkConfigDefault', 'PDSTkConfigDefault', 'STRIDEkConfigDefault', 'BiTRRTkConfigDefault', 'LBTRRTkConfigDefault', 'BiESTkConfigDefault', 'ProjESTkConfigDefault', 'LazyPRMkConfigDefault', 'LazyPRMstarkConfigDefault', 'SPARSkConfigDefault', 'SPARStwokConfigDefault', 'TrajOptDefault']}}}
            ],
            output='screen'
        )
    return LaunchDescription([
        rvis
    ])

