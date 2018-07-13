#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using namespace std;
using std::vector;


// FusionEKF Constructor function definition
FusionEKF::FusionEKF() {
	is_initialized_ = false;

	previous_timestamp_ = 0;

	// Initializing the matrices;

	R_laser_ = MatrixXd(2, 2);
	R_radar_ = MatrixXd(3, 3);
	H_laser_ = MatrixXd(2, 4);
	Hj_ = MatrixXd(3, 4);

	R_radar_ << 0.09, 0, 0,
		0, 0.0009, 0,
		0, 0, 0.09;

	R_laser_ << 0.0105, 0,
		0, 0.0105;

	H_laser_ << 1, 0, 0, 0,
		0, 1, 0, 0;
	
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1;

	ekf_.P_ = MatrixXd(4, 4);

	// Set the Acceleration noise components

	noise_ax = 9;
	noise_ay = 9;


}

// Destructor function definition
FusionEKF::~FusionEKF() {}


// ProcessMeasurement function definition

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
	if (!is_initialized_)
	{
		ekf_.x_ = VectorXd(4);
		ekf_.x_ << 1, 1, 1, 1;

		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

			ekf_.x_(0) = measurement_pack.raw_measurements_(0)*cos(measurement_pack.raw_measurements_(1));
			ekf_.x_(1) = measurement_pack.raw_measurements_(0)*sin(measurement_pack.raw_measurements_(1));
		}
		else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER){

			ekf_.x_(0) = measurement_pack.raw_measurements_(0);
			ekf_.x_(1) = measurement_pack.raw_measurements_(1);
		}

		ekf_.F_ = MatrixXd(4, 4);
		ekf_.F_ << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

		previous_timestamp_ = measurement_pack.timestamp_;
		is_initialized_ = true;
		return;
	}

	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
	previous_timestamp_ = measurement_pack.timestamp_;

	float dt2 = dt * dt;
	float dt3 = dt2 * dt;
	float dt4 = dt3 * dt;

	// update F matrix to handel varying time steps
	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;

	// Set the Process noise covariance matrix to handel varying velocity
	ekf_.Q_ = MatrixXd(4, 4);

	ekf_.Q_ << dt4 / 4 * noise_ax, 0, dt3 / 2 * noise_ax, 0,
		0, dt4 / 4 * noise_ay, 0, dt3 / 2 * noise_ay,
		dt3 / 2 * noise_ax, 0, dt2*noise_ax, 0,
		0, dt3 / 2 * noise_ay, 0, dt2*noise_ay;

	// Predict
	ekf_.Predict();


	// Update
	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		float x = ekf_.x_(0);
		float y = ekf_.x_(1);
		float vx = ekf_.x_(2);
		float vy = ekf_.x_(3);

		float rho = sqrt(x * x + y * y);
		float theta = atan2(y, x);
		float rho_dot = (x*vx + y * vy) / rho;

		VectorXd z_pred = VectorXd(3);
		z_pred << rho, theta, rho_dot;

		// Change the Kalman filter parameters to Radar
		ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
		ekf_.R_ = R_radar_;

		// Update
		ekf_.UpdateRadar(measurement_pack.raw_measurements_, z_pred);

	
	}
	else {
		// Change the Kalman filter parameters to Laser
		ekf_.H_ = H_laser_;
		ekf_.R_ = R_laser_;

		// Update
		ekf_.UpdateLaser(measurement_pack.raw_measurements_);
	}

}