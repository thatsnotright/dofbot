#include <iostream>

#include "include/dofbot_arm.hpp"

using namespace std;

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
 
  rcl_allocator_t allocator = rcl_get_default_allocator();
   rclc_support_t support;

  // TODO accept the i2c bus id from node parameters
  auto servo_driver = new DofbotServo(1);
  RCLCPP_INFO(LOGGER, "Created Dofbot arm servo driver");
  
   RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
   rcl_node_t node;
   RCCHECK(rclc_node_init_default(&node, "int32_subscriber_rclc", "", &support));

   RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/joint_states"));
  
   // create executor
   rclc_executor_t executor;
   RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
   RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  
   printf("microROS initialised\n");
  
   while(1){
     rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
     usleep(100000);
   }
  
   printf("leaving microROS\n");
   // free resources
   RCCHECK(rcl_subscription_fini(&subscriber, &node));
   RCCHECK(rcl_node_fini(&node));
   
   vTaskDelete(NULL);
  RCLCPP_INFO(LOGGER, "Hello world");
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("dofbot_arm_servo_controller", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  rclcpp::shutdown();
  return 0;
}
