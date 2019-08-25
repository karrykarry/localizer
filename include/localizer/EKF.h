#ifndef _EKF_CLASS_H
#define _EKF_CLASS_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <Eigen/Core>

using namespace Eigen;

class EKF{
	
public:

	MatrixXf move(MatrixXf x, MatrixXf u, float dt, float pitch);
	MatrixXf jacobF(MatrixXf x, MatrixXf u, float dt);
	MatrixXf jacobG(MatrixXf x, MatrixXf u, float dt, float pitch);
	MatrixXf jacobV(MatrixXf x, MatrixXf u, float dt, float pitch);
	MatrixXf jacobM(MatrixXf u, double s_input[]);
	MatrixXf jacobH(MatrixXf x);
	MatrixXf h(MatrixXf x);
};


#endif 
