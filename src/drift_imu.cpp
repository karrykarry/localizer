#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>

sensor_msgs::Imu imu_data;

bool received_flag = false;
double OFFSET_YAWRATE = 0.00206676;
double offset_yawrate = 0;

void imu_callback(const sensor_msgs::ImuConstPtr& msg)
{
	imu_data = *msg;
	static bool first_flag = false;
	static double yawrate_ = 0;
	static int imu_count = 0;
	static ros::Time first_time;

	if(!first_flag){
		first_time = msg->header.stamp;
		first_flag = true;
	}

	if((imu_data.header.stamp - first_time) < ros::Duration(5.0)){
		yawrate_ += imu_data.angular_velocity.z;
		std::cout << "=== calibrating ===" << std::endl;
		std::cout << imu_data.header.stamp - first_time  << std::endl;
		imu_count++;
	}
	else{
		offset_yawrate = yawrate_ / (double)imu_count;
		// std::cout << "yawrate :" << offset_yawrate << "[rad/s]" << std::endl;;
		received_flag = true;
	}
}

void not_matching(double yaw_velo, ros::Publisher pub){
	std_msgs::Bool is_notmatch;

	if(yaw_velo < -0.20 || 0.20 < yaw_velo){
		std::cout<<"\r回転なう"<<std::flush;
    	is_notmatch.data = true;
    }
    else {
		std::cout<<"\r-------"<<std::flush;
		is_notmatch.data = false;
	}
	
	pub.publish(is_notmatch);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drift_imu");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  ros::Subscriber imu_sub = nh.subscribe("/imu/data", 100, imu_callback);

  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data/calibrated", 100);
  ros::Publisher notmatching_pub = nh.advertise<std_msgs::Bool>("/not_matching", 100);

  ros::Rate loop_rate(50);

  while(ros::ok()){
	if(received_flag){
	  // imu_data.angular_velocity.z -= OFFSET_YAWRATE;
	  imu_data.angular_velocity.z -= offset_yawrate;
	  imu_pub.publish(imu_data);
		not_matching(imu_data.angular_velocity.z, notmatching_pub);

	  received_flag = false;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
