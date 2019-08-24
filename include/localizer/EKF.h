#ifndef _EKF_CLASS_H
#define _EKF_CLASS_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <Eigen/Core>

//using namespace NEWMAT;


using namespace Eigen;
// Listenerクラス
// Talkerからのデータを受信するcallback関数と、メインループのみのクラス
class EKF{
private:
	
	//nav_msgs::OdometryConstPtr	odo_data;
	//geometry_msgs::PoseStamped	hec_data;
	//geometry_msgs::PoseStamped	localization_data;
	
public:

	//ros::Publisher pub;

	//ros::NodeHandle node_handle_;
	//ros::Subscriber hec_sub;
	//ros::Subscriber odo_sub;

	
	

	//void mainLoop(); // メインループ	
	// 受信時に呼ばれるコールバック関数
	//void ourCallback1(const geometry_msgs::PoseStamped& h_pose);
	//void ourCallback2(const nav_msgs::OdometryConstPtr& odo);
	
	//EKF t
	//t.ekf(input, state, obs)の形にしたい
//	MatrixXf ekf(MatrixXf input, MatrixXf state,MatrixXf obs,float dt, MatrixXf s_input, MatrixXf s_obs);
	MatrixXf move(MatrixXf x, MatrixXf u, float dt, float pitch);
	MatrixXf jacobF(MatrixXf x, MatrixXf u, float dt);
	MatrixXf jacobG(MatrixXf x, MatrixXf u, float dt, float pitch);
	MatrixXf jacobV(MatrixXf x, MatrixXf u, float dt, float pitch);
	MatrixXf jacobM(MatrixXf u, double s_input[]);
	MatrixXf jacobH(MatrixXf x);
	MatrixXf h(MatrixXf x);
//	MatrixXf measurement_update();
	
	//MatrixXf xEst;
	//MatrixXf PEst;
	//MatrixXf Q;
	//MatrixXf H;
	//MatrixXf y;
	//MatrixXf S;
	//MatrixXf R;
	//MatrixXf K;
};


#endif // _LISTENER_CLASS_H
