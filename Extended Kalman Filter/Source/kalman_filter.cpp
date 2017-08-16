#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
# define Pi 3.14159265358979323846

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
  VectorXd z_pred = H_ * x_;
  VectorXd Y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd Pht =  P_ * Ht;
  MatrixXd K = Pht * Si;

  //new state
  x_ = x_ + (K * Y);
  long long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  float rho = sqrt(px*px+py*py);
  float theta = atan2(py,px);
  float rho_dot = 0;
  if(fabs(rho) > 0.0001)
    rho_dot = (px * vx + py * vy)/ rho;
  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, rho_dot;

  VectorXd Y = z - z_pred;
  Y(1) = atan2(sin(Y(1)),cos(Y(1)));
  while (Y(1) < -Pi)
    Y(1) += 2 * Pi;
  while (Y(1) > Pi)
    Y(1) -= 2 * Pi;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd Pht =  P_ * Ht;
  MatrixXd K = Pht * Si;

  //new state
  x_ = x_ + (K * Y);
  long long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
