# Some aliases to quickly get compile source and navigate to project
alias proj="cd ~/ros2_ws/src"
alias cb="cd ~/ros2_ws \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    && proj"

alias sw="source /opt/ros/$ROS_DISTRO/setup.bash && source ~/ros2_ws/install/setup.bash"
