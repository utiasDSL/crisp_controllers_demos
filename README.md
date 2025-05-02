# CRISP controllers Demos

<img src="https://github.com/user-attachments/assets/284983f8-2311-4699-86ab-06fc2ea9d5af" alt="CRISP Controllers Logo" width="160" align="right"/>

<a href="https://github.com/utiasDSL/crisp_controllers/actions/workflows/docker_build.yml"><img src="https://github.com/utiasDSL/crisp_controllers/actions/workflows/docker_build.yml/badge.svg"/></a>
<a href="https://danielsanjosepro.github.io/crisp_controllers/"><img alt="Static Badge" src="https://img.shields.io/badge/docs-passing-blue?style=flat&link=https%3A%2F%2Fdanielsanjosepro.github.io%2Fcrisp_controllers%2F"></a>

This repo provides Docker containers to provide directly test the [crisp_controllers](https://github.com/utiasDSL/crisp_controllers) with real hardware or in simulation with a simple [MuJoCo](https://github.com/google-deepmind/mujoco) `ros2_control` interface provided in this repository.

## Available demos
For now, these are the available demos in this repository. New demos are welcome, in particular if tested with real hardware.
Some other manipulators that couldd be added to this list is the ![Open Manipulator](https://github.com/ROBOTIS-GIT/open_manipulator) or other dual setups.

| Robots | Franka Robotics FR3 | FR Dual FR3 | IIWA 14 | Kinova Gen3 |
| :--- | :---: | :---: | :---: | :---: |
| MuJoCo simulated Hardware | ✅ | ✅ | ✅ | ✅ |
| Real Hardware | ✅ | ✅ | ❔[^1]  | ❌[^2] |

[^1]: Untested, but effort interface available.

[^2]: Might require modifications see https://github.com/Kinovarobotics/ros2_kortex/issues/202

## Getting started


0. (TEMPORARY) Clone the crisp controllers to the repo (later it will be part of the Dockerfile)
```bash
git clone git@github.com:utiasDSL/crisp_controllers.git crisp_controllers
```

1. Build and start the provided container.
```bash
docker compose build
```
2. Start your robot:
```bash
docker compose up launch_iiwa
```
or
```bash
LEFT_ROBOT_IP=172.16.1.2 RIGHT_ROBOT_IP=172.16.0.2 FRANKA_FAKE_HARDWARE=true docker compose up launch_dual_franka
```
or
```bash
ROBOT_IP=172.16.0.2 FRANKA_FAKE_HARDWARE=true docker compose up launch_franka
```
or
```bash
docker compose up launch_kinova
```

3. Now you can publish to `/target_joint` or `/target_pose`! Check [crisp_py](https://github.com/utiasDSL/crisp_py) examples to see how to easily use it.

*TODO also provide an example with interactive markers*

<details>
<summary><h2>How are the manipulors being simulated?</h2></summary>
We implemeted a simple `MujocoHardwareInterface` to simulate the robots. This code is heavily inspired by the simulator in <a href="https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/tree/ros2/cartesian_controller_simulation">cartesian_controllers</a>, but probably better alternatives to use mujoco as a backend simulation would be <a href="https://github.com/moveit/mujoco_ros2_control">mujoco_ros2_control</a>. One could also use gazebo. 
The mujoco files come from the mujoco menagerie and have been slightly modified to use torque based actuators + we added some friction to the joints (to increase realism).
</details>
