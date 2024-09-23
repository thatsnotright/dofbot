#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <iostream>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

using namespace std;

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  RCLCPP_INFO(LOGGER, "Hello world");
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "arm";

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node,
                                                            PLANNING_GROUP);

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose = move_group.getCurrentPose();

  auto current_position = robot_pose.pose;

  /*Retrive position and orientation */
  auto exact_pose = current_position.position;
  auto exact_orientation = current_position.orientation;

  RCLCPP_INFO(LOGGER, "Reference frame : %s",
              move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "Reference frame : %s",
              move_group.getEndEffectorLink().c_str());

  std::cout << "Robot position : " << exact_pose.x << "\t" << exact_pose.y
            << "\t" << exact_pose.z << std::endl;
  std::cout << "Robot Orientation : " << exact_orientation.x << "\t"
            << exact_orientation.y << "\t" << exact_orientation.z << "\t"
            << exact_orientation.w << std::endl;

  move_group.allowReplanning(true);
  move_group.setPlanningTime(20);
  move_group.setNumPlanningAttempts(10);
  move_group.setGoalJointTolerance(.1);
  move_group.setGoalPositionTolerance(.1); // 0.01
  move_group.setGoalOrientationTolerance(.1);
  move_group.setGoalTolerance(.1);;
  move_group.setMaxVelocityScalingFactor(1.0);
  move_group.setMaxAccelerationScalingFactor(1.0);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  for (int i = 0; i < 10; i++) {
      string link = move_group.getEndEffectorLink();
      move_group.setStartStateToCurrentState();
      auto pose = current_position;
      pose.position.x += 0.03;
      pose.orientation.w = 1.0;
      move_group.setPoseTarget(pose, "left_finger");

//      std::vector<double> angles = { .7854, 0, 0.7854, 0, 0 };
//      move_group.setPlanningPipelineId("stomp");
//      move_group.setJointValueTarget(angles);
      RCLCPP_INFO(LOGGER, "Attempt %d", i);
      auto const [success, plan] = [&move_group] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group.move());
        return std::make_pair(ok, msg);
      }();

      RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s",
                success ? "" : "FAILED");
      if (success) {
//        move_group.execute(plan);
        break;
      }
    }

    rclcpp::shutdown();
  return 0;
}
