cd ~/src/Firmware 

source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/sky_ws/src/thales2020/models
roslaunch thales2020 H_world.launch

