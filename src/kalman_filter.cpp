#include <iostream>
#include "kalman_filter.h"
using namespace std;

KalmanFilter::KalmanFilter() {};

//Distructor 
KalmanFilter::~KalmanFilter() {};

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
	MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {

	x_ = x_in;
	P_ = P_in;
	F_ = F_in;
	H_ = H_in;
	R_ = R_in;
	Q_ = Q_in;

}

void KalmanFilter::Predict() {
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::UpdateLaser(const VectorXd &z) {
	
	VectorXd y = z - H_ * x_;

	MatrixXd Ht = H_.transpose();

	MatrixXd S = H_ * P_ * Ht + R_;

	MatrixXd Si = S.inverse();

	MatrixXd K = P_ * Ht * Si;

	x_ = x_ + (K*y);

	long xsize = x_.size();

	MatrixXd I = MatrixXd::Identity(xsize, xsize);

	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateRadar(const VectorXd &z, const VectorXd &z_pred) {
	VectorXd y = z - z_pred;

	// Angle Normalization
	while (y(1) > M_PI) y(1) -= 2 * M_PI;
	while (y(1) < -M_PI) y(1) += 2 * M_PI;

	MatrixXd Ht = H_.transpose();

	MatrixXd S = H_ * P_ * Ht + R_;

	MatrixXd Si = S.inverse();

	MatrixXd K = P_ * Ht * Si;

	x_ = x_ + (K * y);

	long xsize = x_.size();

	MatrixXd I = MatrixXd::Identity(xsize, xsize);

	P_ = (I - K * H_) * P_;

}