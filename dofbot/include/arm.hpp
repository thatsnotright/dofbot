#ifndef __DOFBOT_ARM_INTERFACE__
#define __DOFBOT_ARM_INTERFACE__

#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <fstream>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <hardware_interface/actuator_interface.hpp>
#include "visibility_control.h"

#define DOFBOT_BASE_ADDR 0x15

using namespace std;

namespace dofbot {

class DofbotServo : public hardware_interface::ActuatorInterface {
private:
  int i2c_bus;
  bool simulated;
public:
  DofbotServo();
  ~DofbotServo();
  void set_angle(uint8_t servo_id, double angle, uint16_t time);
  void set_angle(double angles[], uint16_t move_time);

  DOFBOT_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

  DOFBOT_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

  DOFBOT_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

  DOFBOT_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DOFBOT_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  DOFBOT_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

DOFBOT_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  DOFBOT_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

DOFBOT_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
  std::vector<double> joint_angles;
  std::vector<double> joint_command_angles;
  int open_bus(int i2c_bus);
  double read_angle(uint8_t servo_id);
};
} // namespace dof_bot

#endif
