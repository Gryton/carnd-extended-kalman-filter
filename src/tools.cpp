#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	if (estimations.size() != ground_truth.size()
		|| estimations.size() == 0) {
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for (unsigned int i = 0; i < estimations.size(); ++i) {

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse / estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	MatrixXd Hj(3, 4);
	//recover state parameters
	double px = x_state(0);
	double py = x_state(1);
	double vx = x_state(2);
	double vy = x_state(3);
	double c1 = px*px + py*py;
	//TODO: YOUR CODE HERE 

	//check division by zero
	if (fabs(c1) < 0.0001) {
		cout << "\n\nCalculateJacobian () - Error - Division by Zero\n\n" << endl;
		Hj << 0, 0, 0, 0,
			1e+6, 1e+6, 0, 0,
			0, 0, 0, 0;
		return Hj;
	}
	else {
		Hj << px / sqrt(c1), py / sqrt(c1), 0, 0,
			-py / (c1), px / (c1), 0, 0,
			py*(vx*py - vy*px) / pow((c1), 3 / 2), px*(vy*px - vx*py) / pow((c1), 3 / 2), px / sqrt(c1), py / sqrt(c1);
	}
	return Hj;
}
