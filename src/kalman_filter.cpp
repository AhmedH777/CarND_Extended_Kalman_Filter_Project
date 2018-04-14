#include "kalman_filter.h"
#include <iostream>

#define PI 3.14159265

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in) {
  x_ = x_in;
  P_ = P_in;

}

void KalmanFilter::Predict(MatrixXd &F_in, MatrixXd &Q_in) {
  /**
  TODO:
    * predict the state
  */
	x_ = F_in * x_;
	P_ = F_in * P_ * F_in.transpose() + Q_in;
}

void KalmanFilter::Update(const VectorXd &z, MatrixXd &H_in, MatrixXd &R_in) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	long x_size = x_.size();
	MatrixXd S,K;	
	VectorXd y;
	MatrixXd I = MatrixXd::Identity(x_size, x_size);

	y = z - (H_in * x_);

	S = H_in * P_ * H_in.transpose() + R_in;
	K = P_ * H_in.transpose() * S.inverse();

	x_ = x_ + K * y;
	P_ = (I - K * H_in) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z, MatrixXd &R_in) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	long x_size = x_.size();
	MatrixXd S,K,H_in;	
	VectorXd y;
	MatrixXd I = MatrixXd::Identity(x_size, x_size);

	H_in = tools.CalculateJacobian(x_);

	y = z - tools.h(x_);

	cout<<y(0)<<endl;
	cout<<y(1)<<endl;
	cout<<y(2)<<endl;

	while(y(1) < -PI || y(1) > PI)
	{
		if(y(1) < -PI) y(1) += 2 * PI;
		else y(1) -= 2 * PI;
	}
	
	cout<<y(0)<<endl;
	cout<<y(1)<<endl;
	cout<<y(2)<<endl;
	S = H_in * P_ * H_in.transpose() + R_in;
	K = P_ * H_in.transpose() * S.inverse();

	x_ = x_ + K * y;
	P_ = (I - K * H_in) * P_;
}
