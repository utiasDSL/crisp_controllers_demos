export ROS_DOMAIN_ID=100
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export RUST_LOG=zenoh=info


ros2 daemon stop && ros2 daemon start
