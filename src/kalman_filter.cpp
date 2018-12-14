#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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

void KalmanFilter::Predict() 
{
  /**
   * TODO: predict the state
   */
   x_ = F_ * x_;
   P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) 
{
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  //Set the new state
  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) 
{
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
   
  // Find our Z predicition
  // First, pull out the variables we need from the x_ Vector
  // Note that px and py are position in the x and y respectively
  // This change is noted here since x and y in this function are vectors as well
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt(pow(px,2) + pow(py,2));
  float theta = atan2(py,px);
  float rho_dot = ((px * vx + py * vy)/rho);

  VectorXd z_prediction = VectorXd(3);
  z_prediction << rho, theta, rho_dot;
  
  VectorXd y = z - z_prediction;
  // limit y to -pi to pi
  while (y(1) > M_PI)
  {
    y(1) = y(1) - M_PI;
  }
  while(y(1) < -M_PI)
  {
    y(1) = y(1) + M_PI;
  }
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  //Set the new state
  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());
  P_ = (I - K * H_) * P_;
}
