# Mujoco Sim

This package provides a Mujoco system interface for ros2_control. This is a work in progress but it can be used to control a Franka robot with ros2_control.
To start the robot with fake hardware, run the following command:

## Hardware interfaces

For each controller: 
- `command_interfaces`: effort
- `state_interfaces`: position, velocity, effort
