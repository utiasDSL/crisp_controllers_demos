#include "crisp_mujoco_sim/system_interface.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <string>
#include <thread>
#include <vector>

#include "crisp_mujoco_sim/mujoco_simulator.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace crisp_mujoco_sim
{

Simulator::CallbackReturn Simulator::on_init(const hardware_interface::HardwareInfo & info)
{
  // Keep an internal copy of the given configuration
  if (hardware_interface::SystemInterface::on_init(info) != Simulator::CallbackReturn::SUCCESS)
  {
    return Simulator::CallbackReturn::ERROR;
  }

  info_ = info;

  if (info_.joints.size() == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("Simulator"), "No joints found");
    return CallbackReturn::ERROR;
  }

  // Get the clock
  clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

  // Start the simulator in parallel.
  // Let the thread's destructor clean-up all resources
  // once users close the simulation window.
  m_mujoco_model = info_.hardware_parameters["mujoco_model"];
  m_simulation = std::thread(MuJoCoSimulator::simulate, m_mujoco_model);
  m_simulation.detach();

  m_positions.resize(info_.joints.size(), 0.0);
  m_velocities.resize(info_.joints.size(), 0.0);
  m_efforts.resize(info_.joints.size(), 0.0);

  m_effort_commands.resize(info_.joints.size(), 0.0);

  int joint_idx = 0;
  for (const hardware_interface::ComponentInfo & joint : info_.joints){
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Simulator"), "Found joint: " << joint.name);
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(rclcpp::get_logger("Simulator"), "Joint '%s' needs 1 command interface.",
                   joint.name.c_str());
      return Simulator::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_ERROR(rclcpp::get_logger("Simulator"), "Joint '%s' needs the following command interface: %s.",
                   joint.name.c_str(), hardware_interface::HW_IF_EFFORT);
      return Simulator::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_ERROR(rclcpp::get_logger("Simulator"), "Joint '%s' needs 3 state interfaces.",
                   joint.name.c_str());

      return Simulator::CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_ERROR(rclcpp::get_logger("Simulator"),
                   "Joint '%s' needs the following state interfaces in that order: %s, %s, and %s.",
                   joint.name.c_str(), hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);

      return Simulator::CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].initial_value.empty())) {
      m_positions[joint_idx] = joint.state_interfaces[0].initial_value.at(0);
    }
    joint_idx++;
  }

  return Simulator::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Simulator::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &m_positions[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &m_velocities[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &m_efforts[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Simulator::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &m_effort_commands[i]));
  }

  return command_interfaces;
}

Simulator::return_type Simulator::prepare_command_mode_switch(
  [[maybe_unused]] const std::vector<std::string> & start_interfaces,
  [[maybe_unused]] const std::vector<std::string> & stop_interfaces)
{
  return return_type::OK;
}

Simulator::return_type Simulator::read([[maybe_unused]] const rclcpp::Time & time,
                                       [[maybe_unused]] const rclcpp::Duration & period)
{
  /*RCLCPP_INFO_STREAM_THROTTLE(rclcpp::get_logger("Simulator"), *clock, 1000, "Reading state values" << m_positions[0]);*/
  MuJoCoSimulator::getInstance().read(m_positions, m_velocities, m_efforts);
  return return_type::OK;
}

Simulator::return_type Simulator::write([[maybe_unused]] const rclcpp::Time & time,
                                        [[maybe_unused]] const rclcpp::Duration & period)
{
  /*RCLCPP_INFO_STREAM_THROTTLE(rclcpp::get_logger("Simulator"), *clock, 1000, "Writing effort commands" << m_effort_commands[0]);*/
  MuJoCoSimulator::getInstance().write(m_effort_commands);
  return return_type::OK;
}

}  // namespace crisp_mujoco_sim

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(crisp_mujoco_sim::Simulator,
                       hardware_interface::SystemInterface)
