#pragma once

#include <map>
#include <rclcpp/clock.hpp>
#include <thread>

#include "crisp_mujoco_sim/mujoco_simulator.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace crisp_mujoco_sim
{

class Simulator : public hardware_interface::SystemInterface
{
public:
  using return_type = hardware_interface::return_type;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  RCLCPP_SHARED_PTR_DEFINITIONS(Simulator)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Create a ROS clock instance
  rclcpp::Clock::SharedPtr clock;

private:
  // Command buffers for the controllers
  std::vector<double> m_effort_commands;

  // State buffers for the controllers
  std::vector<double> m_positions;
  std::vector<double> m_velocities;
  std::vector<double> m_efforts;

  // Run MuJoCo's solver in a separate thread
  std::thread m_simulation;

  // Parameters
  std::string m_mujoco_model;
};

}  // namespace crisp_mujoco_sim
