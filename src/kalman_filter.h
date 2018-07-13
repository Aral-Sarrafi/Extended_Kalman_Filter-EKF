#ifndef KALMAN_FILTER_
#define KALMAN_FILTER_
#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;

class KalmanFilter
{
public:
	// State Vector
	VectorXd x_;

	// state transition matrix
	MatrixXd  F_;

	// state Covariance Matrix
	MatrixXd  P_;

	// Process noise covariance matrix
	MatrixXd Q_;

	// Measurement model matrix
	MatrixXd H_;

	// Measurement noise covariance matrix
	MatrixXd R_;

	// Constructor
	KalmanFilter();

	//Distructor 
	~KalmanFilter();

	void Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
		MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in);

	void Predict();

	void UpdateLaser(const VectorXd &z);
	void UpdateRadar(const VectorXd &z, const VectorXd &z_pred);


};
#endif // !KALMAN_FILTER_
