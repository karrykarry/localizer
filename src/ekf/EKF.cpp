//========================================================
//2016.10.01 家永編 
//	行列計算のみにして，predictionとmeasurement updateは
//	このライブラリをincludeするソースで作成すること．
//	各行列の変数名は「確率ロボティクス」に合わせて変更
//	
//========================================================
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <time.h>
#include <sys/time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/LU>
#include "EKF.h"


using namespace std;
using namespace Eigen;

float
translate_angle(float a)
{
	while(a<-(M_PI) || a>M_PI){
		if(a>M_PI){
			a = a-2*M_PI;
		}else if(a<-(M_PI)){
			a = a+2*M_PI;
		}
	}
	return a;
}

MatrixXf EKF::move(MatrixXf x, MatrixXf u, float dt, float pitch)//オドメトリの更新式に基づいている
{
	/* 動作予測
	 * x : 状態
	 * u : 制御
	 * dt: 前ステップとの時間差
	 */
	MatrixXf A(3,1);
	MatrixXf I = MatrixXf::Identity(3,3);
	MatrixXf B(3,2);
//	x.coeffRef(2,0) = translate_angle(x.coeffRef(2,0));
	float theta = x.coeffRef(2,0) + u.coeffRef(1,0)*dt/2;
	
	B << dt*cos(pitch)*cos(theta), 0,
		 dt*cos(pitch)*sin(theta), 0,
		 0, dt;
	
	A = I*x + B*u;
	
	return A;
}

MatrixXf EKF::jacobG(MatrixXf x, MatrixXf u, float dt, float pitch)//予測の線形モデル
{
	/* ヤコビ行列
	 * x : 状態
	 * u : 制御
	 * dt: 前ステップとの時間差
	 */
	MatrixXf G(3,3);
//	x.coeffRef(2,0) = translate_angle(x.coeffRef(2,0));
	float b = dt*cos(pitch)*u.coeffRef(0,0);
	float theta = u.coeffRef(1,0)*dt/2 + x.coeffRef(2,0);

/*	G << 1, 0, -b*sin(theta),
		 0, 1, b*cos(theta),
		 0, 0, 1;
*/	G << 1, 0, -b*sin(theta),
		 0, 1, b*cos(theta),
		 0, 0, 1;
	return G;
}


MatrixXf EKF::jacobV(MatrixXf x, MatrixXf u, float dt, float pitch)//状態量の制御量に関するヤコビ行列
{
	/* ヤコビ行列
	 * x : 状態
	 * u : 制御
	 * dt: 前ステップとの時間差
	 */
	MatrixXf V(3,2);
//	x.coeffRef(2,0) = translate_angle(x.coeffRef(2,0));
	float theta = u.coeffRef(1,0)*dt/2 + x.coeffRef(2,0);
	float v = u.coeffRef(0,0);
	
	V << dt*cos(pitch)*cos(theta), (-v*dt*dt)*cos(pitch)*sin(theta)/2,
		 dt*cos(pitch)*sin(theta), (v*dt*dt)*cos(pitch)*cos(theta)/2,
		 0, dt;
		 
	return V;
}


MatrixXf EKF::jacobM(MatrixXf u, double s_input[])//分散共分散行列
{
	/* ヤコビ行列
	 * u : 制御
	 * s_input: 制御系の計測誤差パラメータ
	 */
	MatrixXf M(2,2);
	float v = u.coeffRef(0,0);
	float w = u.coeffRef(1,0);
	float a1 = (float)s_input[0];
	float a2 = (float)s_input[1];
	float a3 = (float)s_input[2];
	float a4 = (float)s_input[3];
	
	M << a1*v*v + a2*w*w, 0,
		 0, a3*v*v + a4*w*w;
		 
	return M;
}




MatrixXf EKF::jacobF(MatrixXf x, MatrixXf u, float dt)
{
	/* ヤコビ行列
	 * x : 状態
	 * u : 制御
	 * dt: 前ステップとの時間差
	 */
	MatrixXf F(3,3);
//	x.coeffRef(2,0) = translate_angle(x.coeffRef(2,0));
	float b = dt*u.coeffRef(0,0);
	float theta = x.coeffRef(2,0); 
	
	F << 1, 0, -b*sin(theta),
		 0, 1, b*cos(theta),
		 0, 0, 1;

	return F;
}




MatrixXf EKF::jacobH(MatrixXf x)//観測モデル
{
	/* ヤコビ行列
	 * x : 状態
	 */
	MatrixXf H(3,3);

	H<<1, 0, 0,
	   0, 1, 0,
	   0, 0, 1;
	 
	 return H;
}

MatrixXf EKF::h(MatrixXf x)
{
	/* 予測値を元に算出される理想的な観測値
	 * x : 状態
	 */
	MatrixXf z(3,1);
	MatrixXf H(3,3);
	
	H << 1,0,0,
		 0,1,0,
		 0,0,1;

	z = H*x;

	return z;
}




