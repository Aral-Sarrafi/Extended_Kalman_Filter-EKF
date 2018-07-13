#include <iostream>
#include "tools.h"

/* Constructor Function*/
Tools::Tools(){}

/*Distructor Function*/
Tools::~Tools() {}

/*RMSE Function*/
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,const vector<VectorXd> &ground_turth) {

	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	if (estimations.size() != ground_turth.size() || estimations.size()==0)
	{
		cout << " Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	for (unsigned int i = 0; i < estimations.size(); ++i)
	{
		VectorXd error = estimations[i] - ground_turth[i];

		error = error.array()*error.array();
		rmse += error;
	}

	rmse = rmse / (estimations.size());
	rmse = rmse.array().sqrt();

	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &state_x) {

	float px = state_x(0);
	float py = state_x(1);
	float vx = state_x(2);
	float vy = state_x(3);

	MatrixXd Jacobian(3, 4);

	//pre-compute a set of terms to avoid repeated calculation
	float c1 = px * px + py * py;
	float c2 = sqrt(c1);
	float c3 = (c1*c2);

	//check division by zero
	if (fabs(c1) < 0.0001) {
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Jacobian;
	}

	//compute the Jacobian matrix
	Jacobian << (px / c2), (py / c2), 0, 0,
		-(py / c1), (px / c1), 0, 0,
		py*(vx*py - vy * px) / c3, px*(px*vy - py * vx) / c3, px / c2, py / c2;

	return Jacobian;
}