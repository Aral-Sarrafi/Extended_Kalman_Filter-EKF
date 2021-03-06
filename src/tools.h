#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using namespace std;

class Tools {

public:

	Tools();

	virtual ~Tools();

	VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

	MatrixXd CalculateJacobian(const VectorXd& x_state);

};
#endif /* TOOLS_H_*/
