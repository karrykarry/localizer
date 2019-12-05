#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/amsl/ros_catkin_ws/devel/setup.bash

# gnome-terminal -e "/opt/ros/kinetic/bin/roscore" --geometry=50x12+0+0 &
# sleep 3.0s
#
# gnome-terminal -e "/opt/ros/kinetic/bin/rosparam set use_sim_time true" --geometry=50x12+0+50 &
# sleep 0.5s
#
# gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch velodyne_pointcloud 32e_points.launch" --geometry=50x12+0+50 &
# sleep 1.5s

gnome-terminal -e "/opt/ros/kinetic/bin/rosrun rviz rviz -d ./config/local.rviz" --geometry=50x12+0+300
sleep 0.5s

# main source
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch localizer map_match.launch" --geometry=50x12+0+550 &
sleep 1.5s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch localizer tf_pub.launch" --geometry=50x12+500+50 &
sleep 0.5s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch localizer ekf.launch" --geometry=50x12+500+300 &
sleep 0.5s
gnome-terminal -e "/opt/ros/kinetic/bin/rosrun localizer drift_imu" --geometry=50x12+500+550 &
sleep 0.5s


