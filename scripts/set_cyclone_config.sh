export ROS_DOMAIN_ID=100
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=$HOME/ros2_ws/src/crisp_controllers_demos/config/cyclone_config.xml

ros2 daemon stop && ros2 daemon start
