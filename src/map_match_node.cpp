/* map_match_node.cpp
 *
 * 2019.08.25
 *
 * author : R.Kusakari
 *
*/ 
#include<ros/ros.h>
#include"map_match.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "map_match");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");
	ros::Rate loop(10);

    ROS_INFO("\033[1;32m---->\033[0m map_match Started.");
	
	Matcher matcher(n,priv_nh);	

	std::string map_file;
	// priv_nh.getParam("map_file",map_file);
	
	map_file = argv[1];
	matcher.map_read(map_file);
	
	while(ros::ok()){
	
		if(matcher.is_start) matcher.process();
		ros::spinOnce();
		loop.sleep();
	}
 
	return 0;
}

       






