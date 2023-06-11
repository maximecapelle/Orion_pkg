#!/bin/bash
if [[ $1 == "config" ]]
then
	cd /WS/orion_ws/
	colcon build
	source install/setup.bash
	ros2 launch orion_pkg pubsublaunch,py
	bash
fi




# elif [[ $1 == "autostart" ]]
# then
# 	source /opt/ros/melodic/setup.bash
# 	cd /root/exomy_ws
# 	catkin_make
# 	http-server src/exomy/gui -p 8000 &

# 	source devel/setup.bash
# 	roslaunch exomy exomy.launch

# 	bash
# elif [[ $1 == "devel" ]]
# then
# 	cd /root/exomy_ws
# 	source /opt/ros/melodic/setup.bash
# 	# catkin_make
# 	# source devel/setup.bash
# 	bash
# else
# 	bash
# fi

