#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

class Tf_publisher{
	private:
		ros::Subscriber odom_sub;

		std::string PARENT_FRAME, CHILD_FRAME;
		
		tf::TransformBroadcaster br;
		tf::Transform transform; 
		
		void prepare();
	public:
		Tf_publisher(ros::NodeHandle n, ros::NodeHandle private_nh_);
		void tf_pub(const geometry_msgs::Pose &p,const ros::Time& current_time);
		
		void odomCallback(const nav_msgs::OdometryConstPtr& msg);
};


Tf_publisher::Tf_publisher(ros::NodeHandle n, ros::NodeHandle private_nh_)
{		
	odom_sub = n.subscribe("/odom", 1, &Tf_publisher::odomCallback, this);

	n.param("PARENT_FRAME", PARENT_FRAME, {"/map"});
	n.param("CHILD_FRAME", CHILD_FRAME, {"/matching_base_link"});
	std::cout<<"PARENT_FRAME : "<<PARENT_FRAME<<std::endl;
	std::cout<<"CHILD_FRAME : "<<CHILD_FRAME<<std::endl;
	prepare();
}


void
Tf_publisher::prepare(){

	transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, 0.0);

	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), PARENT_FRAME, CHILD_FRAME));
}


void
Tf_publisher::tf_pub(const geometry_msgs::Pose &p,const ros::Time& current_time){

	transform.setOrigin( tf::Vector3( p.position.x, p.position.y, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, p.orientation.z);
	// tf::quaternionMsgToTF(p.orientation, q);
    
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, current_time, PARENT_FRAME, CHILD_FRAME));

}

void 
Tf_publisher::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
	tf_pub(msg->pose.pose, msg->header.stamp);
}


int main (int argc, char** argv)
{
	ros::init(argc, argv, "tf_publisher");
	ros::NodeHandle n;
	ros::NodeHandle private_nh_("~");
    ROS_INFO("\033[1;32m---->\033[0m tf_publisher Started.");
	
	Tf_publisher tf_publisher(n, private_nh_);

	ros::spin();

	return (0);
}

