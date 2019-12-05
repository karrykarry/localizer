/* NDT_Dgauss_ekf.cpp 
 * 2016.10.05
 * 
 * author : Takashi Ienaga
 * 
 * Distribution : rwrc16のD-GaussとNDTの拡張カルマンフィルタ(EKF)
 * 				  入力：Gyro-Odometry
 * 				  観測：D-Gauss
 * 					  	NDT
 * を参考に
 */


//Library
#include <ros/ros.h>
#include <tf/tf.h>
#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <sys/time.h>
#include "EKF.h"

//msgs
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

using namespace std;
using namespace Eigen;
EKF ekf;

bool init_pose_flag = false;
bool imu_flag = false;
bool odom_flag = false;
bool ndt_flag = false;


nav_msgs::Odometry ekf_odom;


MatrixXf x(3,1);		// 状態 (x,y,θ)
MatrixXf Sigma(3,3);
MatrixXf u(2,1);		// 制御 (v, w)
MatrixXf obs_ndt(3,1);	// NDT観測 (x,y,θ)

double init_x[3];		// 初期状態 (x,y,θ) [rad]
double init_sig[3];		// 初期分散 (sig_x, sig_y, sig_yaw)
double s_ndt[3]; 		// NDT観測値の分散 (sig_x, sig_y, sig_yaw)
double ndt_sig[2];
double s_input[4]; 		// 制御の誤差パラメータ (要素数[0],[2]は並進速度，[1],[3]は回頭速度のパラメータ)
float pitch;			// ピッチ角



MatrixXf predict(MatrixXf x, MatrixXf u, float dt, double *s_input, float pitch){
	/* u   : (v, w)の転置行列 v:並進速度, w:角速度
	 * x   : (x, y, θ)の転置行列
	 * dt	   : 前ステップからの経過時間
	 * s_input : 動作モデルのノイズパラメータ
	 */
	MatrixXf Mu = MatrixXf::Zero(3,1); //今の位置からのpreのx 
	MatrixXf P = MatrixXf::Zero(3,3);//sigma
	MatrixXf Gt= MatrixXf::Zero(3,3);//線形モデル 偏微分したもの
	MatrixXf Vt= MatrixXf::Zero(3,2);//ヤコビ
	MatrixXf Mt= MatrixXf::Zero(2,2);//分散共分散行列
	
	Mu = x;
	P = Sigma;
	
	Gt = ekf.jacobG(x, u, dt, pitch);
	Vt = ekf.jacobV(x, u, dt, pitch);
	Mt = ekf.jacobM(u, s_input);
	
    Mu = ekf.move(x, u, dt, pitch);
	P = Gt*Sigma*Gt.transpose() + Vt*Mt*Vt.transpose();
	Sigma = P;
	
	return Mu;
}

MatrixXf NDTUpdate(MatrixXf x){
	/* x	: 状態(x, y, yaw)の転置行列
	 * u	: 制御(v, w)の転置行列
	 * s_ndt: 観測ノイズ
	 * sigma: 推定誤差
	 */
	MatrixXf Mu= MatrixXf::Zero(3,1);//predictによる位置
	MatrixXf P = MatrixXf::Zero(3,3);//sigma
	MatrixXf Q= MatrixXf::Zero(3,3);//ndtのRt:共分散行列
	MatrixXf H= MatrixXf::Zero(3,3);//偏微分で求めるヤコビアン
	MatrixXf y= MatrixXf::Zero(3,1);//predictによる現在地とmeasurementによる推定値の差
	MatrixXf S= MatrixXf::Zero(3,3);//観測残差による共分散行列
	MatrixXf K= MatrixXf::Zero(3,3);//最適カルマンゲイン
	MatrixXf I = MatrixXf::Identity(3,3);//単位行列
	
	Mu = x; 
	P = Sigma;
	
    Q.coeffRef(0,0) = (float)s_ndt[0];
	Q.coeffRef(1,1) = (float)s_ndt[1];
	Q.coeffRef(2,2) = (float)s_ndt[2];

	y = obs_ndt - ekf.h(x);	//predictによる現在地とmeasurementによる推定値の差
	
    H = ekf.jacobH(x);

    S = H * Sigma * H.transpose() + Q;
	K = Sigma * H.transpose() * S.inverse();
	Mu = Mu + K*y;
	P = (I - K*H)*Sigma;
	Sigma = P;
	
	return Mu;
}



float expand(float after){

	static bool init_imu = true;
	static float before = 0.000001;
	static float sum;

    if(init_imu){
        before   = after;
        sum      = before;
        init_imu = false; 
    }

    else{
        if((before * after) < 0){

            if(fabs(before) > M_PI/2){ //180度付近
                if(before > 0){
                    sum += (M_PI*2 - before + after);
                }
                else{
                    sum -= (M_PI*2 + before - after);
                }
            }
            else{
                sum += (before - after);
            }
        }

        else{
            sum += (after - before);
        }

        before = after;
    }

    return sum;
}



void odomCallback(nav_msgs::Odometry msg){
	u.coeffRef(0,0) = msg.twist.twist.linear.x;
    
    ekf_odom.twist.twist.linear.x = u.coeffRef(0,0);
	
	odom_flag = true;
}


void imuCallback(sensor_msgs::Imu::ConstPtr msg){
    u.coeffRef(1,0) = msg->angular_velocity.z;

	ekf_odom.twist.twist.angular.z = u.coeffRef(1,0);
	
	pitch = 0;
	imu_flag = true;
	ekf_odom.header.stamp = msg->header.stamp; //

}


void ndtCallback(nav_msgs::Odometry msg){
	// ekf_odom.header.stamp = msg.header.stamp; //
	
    obs_ndt.coeffRef(0,0) = msg.pose.pose.position.x;
	obs_ndt.coeffRef(1,0) = msg.pose.pose.position.y;
	ekf_odom.pose.pose.position.z = msg.pose.pose.position.z;

	float yaw_true = expand(msg.pose.pose.orientation.z);
	obs_ndt.coeffRef(2,0) = yaw_true;

    ndt_flag = true; 
}

void hanteiCallback(const std_msgs::BoolConstPtr msg){
	
	if(msg->data){
        s_ndt[0] = 100;
		s_ndt[1] = 100;
		s_ndt[2] = 100;
    }


    else{
        //徐々に減らしている。
		// s_ndt[0] = s_ndt[0] * 0.5;
		// s_ndt[1] = s_ndt[1] * 0.5;
		// s_ndt[2] = s_ndt[2] * 0.5;
		s_ndt[0] = 0.001;
		s_ndt[1] = 0.001;
		s_ndt[2] = 0.001;

        if(s_ndt[0] < 0.001){
		    s_ndt[0] = 0.001;
		    s_ndt[1] = 0.001;
		    s_ndt[2] = 0.001;
        }
    }
	printf("NDT sig : %.4f\n", s_ndt[0]);
}


void initposeCallback(const geometry_msgs::PoseStampedConstPtr& msg){

	static bool init_flag_ = true;

	if(init_flag_){

		double qr,qp,qy;
		tf::Quaternion quat(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
		tf::Matrix3x3(quat).getRPY(qr, qp, qy);   


		init_x[0] = msg->pose.position.x;
		init_x[1] = msg->pose.position.y;
		init_x[2] = qy;

		x << init_x[0], init_x[1], init_x[2];

		obs_ndt.coeffRef(0,0) = init_x[0];
		obs_ndt.coeffRef(1,0) = init_x[1];
		obs_ndt.coeffRef(2,0) = init_x[2];

		init_flag_ = false;

	}

	init_pose_flag = true;
}




void poseInit(nav_msgs::Odometry &msg){
	msg.header.frame_id = "/map";
	msg.child_frame_id = "/matching_base_link";
	msg.pose.pose.position.x = init_x[0];
	msg.pose.pose.position.y = init_x[1];
	msg.pose.pose.position.z = 0.0; 
	msg.pose.pose.orientation.x = 0.0;
	msg.pose.pose.orientation.y = 0.0;
	msg.pose.pose.orientation.z = init_x[2];
	msg.pose.pose.orientation.w = 0.0;
	
	x << init_x[0], init_x[1], init_x[2];
	Sigma << init_sig[0], 0, 0,
			 0, init_sig[1], 0,
			 0, 0, init_sig[2];
	u = MatrixXf::Zero(2,1);
	obs_ndt = MatrixXf::Zero(3,1);
}



void printParam(void){
	printf("Dgauss_ekf.cpp Parameters:\n");
	printf("Initial pose \n");
	printf("	init_x		: %lf\n", init_x[0]);
	printf("	init_y		: %lf\n", init_x[1]);
	printf("	init_yaw	: %lf\n", init_x[2]);
	printf("	init_sig_x	: %f\n", init_sig[0]);
	printf("	init_sig_y	: %f\n", init_sig[1]);
	printf("	init_sig_yaw	: %f\n", init_sig[2]);
	printf("Prediction \n");
	for(unsigned int i=0; i<sizeof(s_input)/sizeof(s_input[0]); i++){
		printf("	a%d		: %lf\n", i+1, s_input[i]);
	}
	printf("NDT Measurement \n");
	printf("	Sig_X		: %lf\n", s_ndt[0]);	ndt_sig[0] = s_ndt[0];
	printf("	Sig_Y		: %lf\n", s_ndt[1]);	ndt_sig[1] = s_ndt[1];
	printf("	Sig_Yaw		: %lf\n", s_ndt[2]);
}



int main(int argc, char** argv){
	ros::init(argc, argv, "ekf");
	ros::NodeHandle n;
	ros::NodeHandle pnh("~");
    ROS_INFO("\033[1;32m---->\033[0m EKF Started.");
	
	//Subscribe
	ros::Subscriber odom_sub  = n.subscribe("/odom", 10, odomCallback);
	ros::Subscriber imu_sub   = n.subscribe("/imu/data", 10, imuCallback);
	ros::Subscriber ndt_sub   = n.subscribe("/NDT/result", 10, ndtCallback);//ndtによる結果	
	ros::Subscriber hantei_sub = n.subscribe("/not_matching", 1, hanteiCallback);
	ros::Subscriber init_sub    = n.subscribe("/move_base_simple/goal", 1, initposeCallback);
    
    //Publish
	ros::Publisher ekf_pub = n.advertise<nav_msgs::Odometry>("/EKF/result", 100);
	ros::Publisher vis_ekf_pub = n.advertise<nav_msgs::Odometry>("/vis/odometry", 100);

	float dt;
	struct timeval last_time, now_time;
	
	//パラメータ
	pnh.param<double>("/init_x", init_x[0], 0.0);
	pnh.param<double>("/init_y", init_x[1], 0.0);
	pnh.param<double>("/init_yaw", init_x[2], -30.0);
	pnh.param<double>("init_sig_x", init_sig[0], 0.0);
	pnh.param<double>("init_sig_y", init_sig[1], 0.0);
	pnh.param<double>("init_sig_yaw", init_sig[2], 0.0);
	pnh.param<double>("Pred_a1", s_input[0], 0.0);
	pnh.param<double>("Pred_a2", s_input[1], 0.0);
	pnh.param<double>("Pred_a3", s_input[2], 0.0);
	pnh.param<double>("Pred_a4", s_input[3], 0.0);
	pnh.param<double>("NDT_sig_X", s_ndt[0], 0.0);
	pnh.param<double>("NDT_sig_Y", s_ndt[1], 0.0);
	pnh.param<double>("NDT_sig_Yaw", s_ndt[2], 0.0);

	printParam();
	
	//初期化
	poseInit(ekf_odom);
	static bool init_flag = true;
	nav_msgs::Odometry vis_ekf;
	
	//時刻取得
	gettimeofday(&last_time, NULL);
	
	ros::Rate loop(50);
	while(ros::ok()){
		if(init_pose_flag){
			if(imu_flag && odom_flag){
				if(init_flag){
					//時刻取得
					gettimeofday(&now_time, NULL);
					dt = 0.02;
					init_flag = false;
				}else{
					//時刻取得
					gettimeofday(&now_time, NULL);
					dt = (now_time.tv_sec + now_time.tv_usec*1.0e-6) - (last_time.tv_sec + last_time.tv_usec*1.0e-6);
				}
				x = predict(x, u, dt, s_input, pitch);

				if(ndt_flag){
					x= NDTUpdate(x);
				}

				last_time = now_time;
				imu_flag = odom_flag = ndt_flag = false;

			}

			ekf_odom.pose.pose.position.x = x.coeffRef(0,0);
			ekf_odom.pose.pose.position.y = x.coeffRef(1,0);
			ekf_odom.pose.pose.orientation.z = x.coeffRef(2,0);
			ekf_odom.pose.pose.orientation.w = 0.0;
			// ekf_odom.header.stamp = ros::Time::now(); //
			ekf_pub.publish(ekf_odom);

			vis_ekf = ekf_odom;
			vis_ekf.pose.pose.orientation.z = sin(x.coeffRef(2,0) * 0.5);
			vis_ekf.pose.pose.orientation.w = cos(x.coeffRef(2,0) * 0.5);
			vis_ekf_pub.publish(vis_ekf);
		}

		loop.sleep();
		ros::spinOnce();
	}
	
	return 0;
}
