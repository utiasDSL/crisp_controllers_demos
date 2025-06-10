export ROS_DOMAIN_ID=100
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CONFIG_URI=$HOME/ros2_ws/src/crisp_controllers_demos/zenoh_router_config.json5
ros2 daemon stop && ros2 daemon start
