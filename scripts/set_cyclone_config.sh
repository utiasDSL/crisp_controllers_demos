export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [ -z "$ROS_NETWORK_INTERFACE" ]; then
    echo "ROS_NETWORK_INTERFACE is not set. Defaulting to 'lo' (loopback interface)."
    export ROS_NETWORK_INTERFACE=lo
fi

export CYCLONEDDS_URI=file:///home/ros/ros2_ws/src/crisp_controllers_demos/config/cyclone_config.xml

ros2 daemon stop && ros2 daemon start
