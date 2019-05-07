# G-ROSSoftware
All code in relation to ROS development

For env.sh under devel:

#!/bin/bash

export ROS_MASTER_URI="http://192.168.1.101:11311"
export ROS_IP="192.168.1.102"

source /home/pi/ros_catkin_ws/devel/setup.bash

exec "$@"
