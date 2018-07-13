#ifndef FUSIONEKF_H_
#define FUSIONEKF_H_
#include "kalman_filter.h"
#include "tools.h"
#include <vector>
#include <string>
#include <fstream>
#include "measurement_package.h"
class FusionEKF {

public:

	// Constructor
	FusionEKF();

	// Destructor
	virtual ~FusionEKF();

	void ProcessMeasurement(const MeasurementPackage &measurement_pack);

	// This class includes a kalman filter object for processing the measurements
	KalmanFilter ekf_;


private:

	bool is_initialized_;

	long previous_timestamp_;

	// This class includes a tools object for calculating the Jacobian and RMSE
	Tools tools;

	MatrixXd R_laser_;
	MatrixXd R_radar_;
	MatrixXd H_laser_;
	MatrixXd Hj_;

	// Acceleration noise components
	float noise_ax = 9;
	float noise_ay = 9;

};


#endif // !FUSIONEKF_H_

