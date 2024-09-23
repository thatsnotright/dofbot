#include "arm.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <bit>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <unistd.h>
}
#include <math.h>
#include <errno.h>
#define NUM_JOINTS 7


inline double to_degrees(double radians) {
    return radians * (180.0 / M_PI) + 90.;
}

inline double to_radians(double degrees) {
    return (degrees - 90) * (M_PI / 180.0);
}

static const rclcpp::Logger LOGGER = rclcpp::get_logger("DOfBOT");
using namespace std;
using namespace hardware_interface;

namespace dofbot {
DofbotServo::DofbotServo() {
  RCLCPP_INFO(LOGGER, "DOFBOT loaded");
}
DofbotServo::~DofbotServo() {
  RCLCPP_INFO(LOGGER, "DOFBOT unloaded");
  if (i2c_bus) {
//    i2c_smbus_write_byte_data((i2c_bus), 0x1a, 0);
    close(i2c_bus);
  }
}

/**
 * \param[in] hardware_info structure with data from URDF.
 * \returns CallbackReturn::SUCCESS if required data are provided and can be
 * parsed. \returns CallbackReturn::ERROR if any error happens or data are
 * missing.
 */
CallbackReturn DofbotServo::on_init(const HardwareInfo &hardware_info) {
  hardware_interface::ActuatorInterface::on_init(hardware_info);
  std::string SIMULATE{"simulate"};
  simulated = false;
  double angles[] = { 0, 0, 0, 0, 0, 0 };  
  try {
    if (!simulated) {
      i2c_bus = open_bus(1);
    }
  } catch(std::exception e) {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(LOGGER, "DOFBOT Online! Mode: %s, joints %ld",
              simulated ? "Simulated" : "Real", NUM_JOINTS);

  set_angle(angles, 500);
  
  info_ = hardware_info;
  joint_angles.assign(NUM_JOINTS, 0.);
  joint_command_angles.assign(NUM_JOINTS, 0.);
  for (size_t i = 0; i < NUM_JOINTS - 1; i++) {
    double angle = read_angle(i);
    joint_angles[i] = to_radians(angle);
    RCLCPP_INFO(LOGGER, "angle at joint %d: %f %f\n", i, joint_angles[i], to_degrees(joint_angles[i]));
  }
  return CallbackReturn::SUCCESS;
};

CallbackReturn
DofbotServo::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
  if (!simulated)
    close(i2c_bus);
  return CallbackReturn::SUCCESS;
}

// on_configure step 4.4
// https://control.ros.org/master/doc/ros2_controllers/doc/writing_new_controller.html
//  actuator_interface:
//  https://control.ros.org/galactic/doc/api/classhardware__interface_1_1ActuatorInterface.html#details

CallbackReturn
DofbotServo::on_configure(const rclcpp_lifecycle::State &previous_state) {
  RCLCPP_INFO(LOGGER, "DOFBOT configured with %ld joints", NUM_JOINTS);
  return CallbackReturn::SUCCESS;
}

/// Exports all state interfaces for this hardware interface.
/**
 * The state interfaces have to be created and transferred according
 * to the hardware info passed in for the configuration.
 *
 * Note the ownership over the state interfaces is transferred to the caller.
 *
 * \return vector of state interfaces
 */
std::vector<StateInterface> DofbotServo::export_state_interfaces() {
  std::vector<StateInterface> state_interfaces;
  for (size_t i = 0; i < NUM_JOINTS - 1; i++) {
    RCLCPP_INFO(LOGGER, "joint %ld name: %s", i, info_.joints[i].name.c_str());
    state_interfaces.emplace_back(
        StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
                       &joint_angles[i]));
    RCLCPP_INFO(LOGGER, "Joint angle: %ld / %f ", i, joint_angles[i]);
  }
  state_interfaces.emplace_back(
        StateInterface("gripper", hardware_interface::HW_IF_POSITION,
                       &joint_angles[5]));
  state_interfaces.emplace_back(
        StateInterface("gripper_right", hardware_interface::HW_IF_POSITION,
                       &joint_angles[6]));

  return state_interfaces;
}

/// Exports all command interfaces for this hardware interface.
/**
 * The command interfaces have to be created and transferred according
 * to the hardware info passed in for the configuration.
 *
 * Note the ownership over the state interfaces is transferred to the caller.
 *
 * \return vector of command interfaces
 */
std::vector<CommandInterface> DofbotServo::export_command_interfaces() {
  std::vector<CommandInterface> command_interfaces;
  for (size_t i = 0; i < NUM_JOINTS - 1; i++) {
    command_interfaces.emplace_back(CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &joint_command_angles[i]));
  }
    command_interfaces.emplace_back(CommandInterface(
        "gripper", hardware_interface::HW_IF_POSITION,
        &joint_command_angles[5]));
    command_interfaces.emplace_back(CommandInterface(
        "gripper_right", hardware_interface::HW_IF_POSITION,
        &joint_command_angles[6]));

  return command_interfaces;
}

/// Prepare for a new command interface switch.
/**
 * Prepare for any mode-switching required by the new command interface
 * combination.
 *
 * \note This is a non-realtime evaluation of whether a set of command interface
 * claims are possible, and call to start preparing data structures for the
 * upcoming switch that will occur. \note All starting and stopping interface
 * keys are passed to all components, so the function should return
 * return_type::OK by default when given interface keys not relevant for this
 * component. \param[in] start_interfaces vector of string identifiers for the
 * command interfaces starting. \param[in] stop_interfaces vector of string
 * identifiers for the command interfacs stopping. \return return_type::OK if
 * the new command interface combination can be prepared, or if the interface
 * key is not relevant to this system. Returns return_type::ERROR otherwise.
 */
return_type prepare_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/) {
  return return_type::OK;
}

// Perform switching to the new command interface.
/**
 * Perform the mode-switching for the new command interface combination.
 *
 * \note This is part of the realtime update loop, and should be fast.
 * \note All starting and stopping interface keys are passed to all components,
 * so the function should return return_type::OK by default when given interface
 * keys not relevant for this component. \param[in] start_interfaces vector of
 * string identifiers for the command interfaces starting. \param[in]
 * stop_interfaces vector of string identifiers for the command interfacs
 * stopping. \return return_type::OK if the new command interface combination
 * can be switched to, or if the interface key is not relevant to this system.
 * Returns return_type::ERROR otherwise.
 */
return_type perform_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/) {
  return return_type::OK;
}

// on_activate step 4.6
// https://control.ros.org/master/doc/ros2_controllers/doc/writing_new_controller.html

CallbackReturn
DofbotServo::on_activate(const rclcpp_lifecycle::State &previous_state) {
   // https://control.ros.org/galactic/doc/api/classhardware__interface_1_1ActuatorInterface.html#details
  //HW_IF_POSITION;
  //HW_IF_VELOCITY;
  //HW_IF_ACCELERATION;
  //HW_IF_EFFORT;
  // publisher_ -> on_activate(); // ref
  // https://medium.com/@nullbyte.in/ros2-from-the-ground-up-part-8-simplify-robotic-software-components-management-with-ros2-5fafa2738700
  //  and ref
  //  https://answers.ros.org/question/365817/spam-from-the-controller_serverlifecyclepublisher/
  return CallbackReturn::SUCCESS;
}

// on_deactivate step 4.7
// https://control.ros.org/master/doc/ros2_controllers/doc/writing_new_controller.html

CallbackReturn
DofbotServo::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
  // disable torque control
//  i2c_smbus_write_byte_data((i2c_bus), 0x1a, 0);
  if (!simulated)
    close(i2c_bus);

  return CallbackReturn::SUCCESS;
}

/// Read the current state values from the actuator.
/**
 * The data readings from the physical hardware has to be updated
 * and reflected accordingly in the exported state interfaces.
 * That is, the data pointed by the interfaces shall be updated.
 *
 * \param[in] time The time at the start of this control loop iteration
 * \param[in] period The measured time taken by the last control loop iteration
 * \return return_type::OK if the read was successful, return_type::ERROR
 * otherwise.
 */
return_type DofbotServo::read(const rclcpp::Time &time,
                              const rclcpp::Duration &period) {
  for (size_t i = 0; i < NUM_JOINTS - 2; i++) {
    double val = read_angle(i);
    joint_angles[i] = to_radians(val);
  }
/*  RCLCPP_INFO(LOGGER, "read angles %f %f %f %f %f %f", 
		  joint_angles[0],
		  joint_angles[1],
		  joint_angles[2],
		  joint_angles[3],
		  joint_angles[4],
		  joint_angles[5]);*/
  return return_type::OK;
}

/// Write the current command values to the actuator.
/**
 * The physical hardware shall be updated with the latest value from
 * the exported command interfaces.
 *
 * \param[in] time The time at the start of this control loop iteration
 * \param[in] period The measured time taken by the last control loop iteration
 * \return return_type::OK if the write was successful, return_type::ERROR
 * otherwise.
 */
return_type DofbotServo::write(const rclcpp::Time &time,
                               const rclcpp::Duration &period) {
  if (simulated) {
    for (size_t i = 0; i < NUM_JOINTS - 2; i++) {
      joint_angles[i] = joint_command_angles[i];
    }
    return return_type::OK;
  }
/*  RCLCPP_INFO(LOGGER, "set angles %f %f %f %f %f %f", 
		  joint_command_angles[0],
		  joint_command_angles[1],
		  joint_command_angles[2],
		  joint_command_angles[3],
		  joint_command_angles[4],
		  joint_command_angles[5]);
*/
  set_angle(&joint_command_angles[0], 500);

  return return_type::OK;
}

void DofbotServo::set_angle(double angles[], uint16_t move_time) {
  uint8_t data[13] = {0x1d, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  uint8_t time[3] = {0x1e, 0, 0};
  double pos = angles[0];
  uint8_t offset = 1;

  pos = (3100.-900.)/180. * to_degrees(angles[0]) + 900.;
  data[offset+0] = ((uint16_t)pos >> 8) & 0xFF;
  data[offset+1] = (uint16_t)pos & 0xFF;

  pos = 180 - to_degrees(angles[1]);
  pos = (2200 * pos / 180) + 900;
  data[offset+2] = (int)pos >> 8 & 0xFF;
  data[offset+3] = (int)pos & 0xFF;

  pos = 180 - to_degrees(angles[2]);
  pos = (2200 * pos / 180) + 900;
  data[offset+4] = ((int)pos >> 8) & 0xFF;
  data[offset+5] = (int)pos & 0xFF;

  pos = 180 - to_degrees(angles[3]);
  pos = (2200 * (pos) / 180) + 900;
  data[offset+6] = ((int)pos >> 8) & 0xFF;
  data[offset+7] = (int)pos & 0xFF;

  pos = ((3700 - 380) * (to_degrees(angles[4])) / 270) + 380;
  data[offset+8] = ((int)pos >> 8) & 0xFF;
  data[offset+9] = (int)pos & 0xFF;

  pos = (2200 * (to_degrees(angles[5])) / 180) + 900;
  data[offset+10] = ((int)pos >> 8) & 0xFF;
  data[offset+11] = (int)pos & 0xFF;

  time[1] = (move_time >> 8) & 0xFF;
  time[2] = move_time & 0xFF;
  
  if (::write((i2c_bus), time, 3) != 3) {
  //  RCLCPP_INFO(LOGGER, "failed to write time to i2c device");
  }
  if (::write((i2c_bus), data, 13) != 13) {
  //  RCLCPP_INFO(LOGGER, "failed to write data to i2c device");
  }
}

int DofbotServo::open_bus(int i2c_bus) {
  std::string filename = "/dev/i2c-" + std::to_string(i2c_bus);
  int file = open(filename.c_str(), O_RDWR);

  RCLCPP_INFO(LOGGER, "Opening i2c bus: %s", filename.c_str());
  if (file < 0) {
    throw std::runtime_error("Failed to open the i2c bus");
  }

  uint8_t fan_on[] = { 0x0d, 0x08, 0x08 };
  if (::write(i2c_bus, fan_on, 3) != 3) {
    RCLCPP_INFO(LOGGER, "failed to write fan on to i2c device");
  }



  if (ioctl(file, I2C_SLAVE_FORCE, 0x15) < 0) {
    throw std::runtime_error("Failed to set i2c bus address");
  }
  return file;
}

double DofbotServo::read_angle(uint8_t servo_id) {
  if (simulated) {
    return joint_angles[servo_id];
  }
  uint16_t raw_angle = 0;
  uint8_t tries = 4;
  while(raw_angle < 900 && tries-- > 0) {
    i2c_smbus_write_byte_data((i2c_bus), 0x31 + servo_id, 0);
    usleep(3000);
    raw_angle = i2c_smbus_read_word_data((i2c_bus), 0x31 + servo_id);
    raw_angle = (raw_angle & 0xFF00) >> 8 | (raw_angle & 0xFF) << 8;
  }
  double angle = 0.;
  if (servo_id == 4) {
    // adjust the angle for the gripper
    // the gripper has a range of 0 to 270 degrees
    angle = 270. * ((raw_angle - 380.) / 3320.);
    if (angle < 0. || angle > 270.) {
      return 0;
    }
  } else {
    angle = 180. * ((raw_angle - 900.) / 2200.);
  }
  if (servo_id > 0 && servo_id < 4) {
    angle = 180. - angle;
  }
  return angle;
}

} // namespace dofbot

// namespace dofbot
PLUGINLIB_EXPORT_CLASS(dofbot::DofbotServo,
                       hardware_interface::ActuatorInterface)
