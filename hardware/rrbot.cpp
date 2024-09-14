// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros2_control_demo_example_1/rrbot.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <cmath>  // For M_PI

// Includes for serial communication
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iomanip>  // For std::setprecision
#include <sstream>  // For std::ostringstream

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_1
{
hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
serial_port_ = ::open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_port_ == -1)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
      "Failed to open serial port /dev/ttyUSB0. Error: %s", strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Configure serial port
  memset(&tty_, 0, sizeof tty_);

  if (tcgetattr(serial_port_, &tty_) != 0)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
      "Error from tcgetattr: %s", strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Set baud rate to 115200
  cfsetospeed(&tty_, B115200);
  cfsetispeed(&tty_, B115200);

  // Configure port settings
  tty_.c_cflag &= ~PARENB;        // No parity bit
  tty_.c_cflag &= ~CSTOPB;        // One stop bit
  tty_.c_cflag &= ~CSIZE;
  tty_.c_cflag |= CS8;            // 8 bits per byte
  tty_.c_cflag &= ~CRTSCTS;       // No hardware flow control
  tty_.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

  tty_.c_lflag &= ~ICANON;
  tty_.c_lflag &= ~ECHO;          // Disable echo
  tty_.c_lflag &= ~ECHOE;         // Disable erasure
  tty_.c_lflag &= ~ECHONL;        // Disable new-line echo
  tty_.c_lflag &= ~ISIG;          // Disable interpretation of INTR, QUIT and SUSP
  tty_.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off software flow control
  tty_.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  tty_.c_oflag &= ~OPOST;         // Prevent special interpretation of output bytes
  tty_.c_oflag &= ~ONLCR;         // Prevent conversion of newline to carriage return/line feed

  // Set read timeout
  tty_.c_cc[VMIN] = 0;
  tty_.c_cc[VTIME] = 10; // 1 second read timeout

  if (tcsetattr(serial_port_, TCSANOW, &tty_) != 0)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
      "Error from tcsetattr: %s", strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Serial port configured");

  // reset values always when configuring hardware
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = 0;
    hw_commands_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RRBotSystemPositionOnlyHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RRBotSystemPositionOnlyHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Activating ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_commands_[i] = hw_states_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Deactivating ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "%.1f seconds left...",
      hw_stop_sec_ - i);
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotSystemPositionOnlyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  char buf[256];
  ssize_t num_bytes = ::read(serial_port_, buf, sizeof(buf));

  if (num_bytes < 0)
  {
    if (errno == EAGAIN || errno == EWOULDBLOCK)
    {
      // No data available right now, but it's not an error
      RCLCPP_DEBUG(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "No data available on serial port right now.");
      return hardware_interface::return_type::OK;
    }
    else
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Error reading from serial port: %s", strerror(errno));
      return hardware_interface::return_type::ERROR;
    }
  }
  else if (num_bytes == 0)
  {
    RCLCPP_DEBUG(
      rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
      "No data read from serial port");
  }
  else
  {
    // Append received data to the read buffer
    read_buffer_.append(buf, num_bytes);

    // Process complete lines (terminated by '\n')
    size_t pos = 0;
    while ((pos = read_buffer_.find('\n')) != std::string::npos)
    {
      std::string line = read_buffer_.substr(0, pos);
      read_buffer_.erase(0, pos + 1); // Remove the processed line from the buffer

      // Remove any carriage return characters
      line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());

      // Ensure the line is not empty
      if (line.empty())
      {
        RCLCPP_WARN(
          rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
          "Received empty line, skipping.");
        continue;  // Skip empty lines
      }

      // Now, try to parse the line as a floating-point angle
      try
      {
        float angle = std::stof(line);
        if (angle >= -M_PI_2 && angle <= M_PI_2)  // Ensure angle is in the valid range
        {
          hw_states_[0] = static_cast<double>(angle);  // Store the angle in hw_states_
          RCLCPP_INFO(
            rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
            "Read angle: %.2f", angle);
        }
        else
        {
          RCLCPP_WARN(
            rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
            "Received angle out of range: %.2f", angle);
        }
      }
      catch (const std::invalid_argument & e)
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
          "Invalid data received: '%s'", line.c_str());
      }
      catch (const std::out_of_range & e)
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
          "Received angle is out of range: '%s'", line.c_str());
      }
    }
  }

  // Even though we have removed the simulation, we must still update the state of the joint
  // Ensure joint state is updated with new data from the servo

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotSystemPositionOnlyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

    if (hw_commands_.empty())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
      "No command available to write.");
    return hardware_interface::return_type::ERROR;
  }

  // Get the command value
  float normalizedValue = static_cast<float>(hw_commands_[0]);

if (normalizedValue < -M_PI_2 || normalizedValue > M_PI_2)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
      "Invalid command value: %.3f. Must be between -π/2 and π/2.", normalizedValue);
    return hardware_interface::return_type::ERROR;
  } 
 
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Writing...");

  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_1

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_1::RRBotSystemPositionOnlyHardware, hardware_interface::SystemInterface)