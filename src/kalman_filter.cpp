#include "kalman_filter.h"
#include "tools.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

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
  /**
  TODO:
    * predict the state
  */
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	std::cout << "lidar update" << std::endl;
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_lidar;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	double const pi = 3.1415926535897;
	std::cout << "radar update" << std::endl;
	double p_x = x_(0);
	double p_y = x_(1);
	double vx = x_(2);
	double vy = x_(3);
	double meas_x = z(0) * std::cos(z(1));
	double meas_y = z(0) * std::sin(z(1));
	Tools tools = Tools();
	MatrixXd Hj_ = tools.CalculateJacobian(x_);
	// calculating predicted state for radar via h(x')
	VectorXd z_pred(3);
	double range = sqrt(p_x*p_x + p_y*p_y);
	double phi = std::atan2(p_y,  p_x);
	z_pred << range, phi, (p_x*vx + p_y*vy) / range;
	std::cout << "angle: " << phi << std::endl;
	std::cout << "py / px = " << p_y / p_x << std::endl;
	VectorXd y = z - z_pred;
	if (y(1) > pi)
		y(1) -= 2 * pi;
	if (y(1) < -pi)
		y(1) += pi;
	MatrixXd Ht = Hj_.transpose();
	MatrixXd S = Hj_ * P_ * Ht + R_radar;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;
	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * Hj_) * P_;
}
