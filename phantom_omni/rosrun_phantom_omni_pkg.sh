 #!/bin/bash  
export ROS_IP=10.190.12.40
export ROS_MASTER_URI=http://10.190.12.40:11311

source /opt/ros/indigo/setup.bash
source ~/catkin_ws/devel/setup.bash

rosrun phantom_omni_pkg phantom_omni_pkg_node