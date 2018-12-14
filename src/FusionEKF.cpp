#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  // The H matrix for lasers needs to discard velocty information
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.P_ = MatrixXd(4, 4);
  // The Udacity Quiz Provides Noise_ax and Noise_ay as 9
  // This value is usually provided by the sensor manufacturer
  // So this is not something we would normally calculate
  noise_ax_ = acceleration_noise;
  noise_ay_ = acceleration_noise;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
  * Initialization
  */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "Initializing EKF ProcessMeasurement!" << endl;
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
    {
      // Radar gives us polar coordinates, so we must take those 
      // and convert them to cartesian using trig
      float rho = measurement_pack.raw_measurements_[0];
      float theta = measurement_pack.raw_measurements_[1];
      ekf_.x_(0) = rho * cos(theta);
      ekf_.x_(1) = rho * sin(theta);

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) 
    {
      // Lidar/laser gives us exactly what we need here, so there is no need to
      // convert.
      float x = measurement_pack.raw_measurements_[0];
      float y = measurement_pack.raw_measurements_[1];
      ekf_.x_(0) = x;
      ekf_.x_(1) = y;
    }
  //initialize F
  // Note that dt here is 0, since we are at the beginning
  // thus dt = 0 and this gets initialized to a 1 diagonal Matrix
  ekf_.F_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
  // set up the new previous_timestamp
  previous_timestamp_ = measurement_pack.timestamp_;
  // done initializing, no need to predict or update
  is_initialized_ = true;
  return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  // First, get the time and convert it to seconds
  float dt = measurement_pack.timestamp_ - previous_timestamp_ / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  //put the time into the F matrix
  ekf_.F_ << 1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;

  //Next Set up the Covariance Q Matrix
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << (pow(dt,4)*noise_ax_)/4, 0, (pow(dt,3)*noise_ax_)/2, 0,
            0, (pow(dt,4)*noise_ay_)/4, 0, (pow(dt,3)*noise_ay_)/2,
            (pow(dt,3)*noise_ax_)/2, 0, (pow(dt,2)*noise_ax_), 0,
            0, (pow(dt,3)*noise_ay_)/2, 0, (pow(dt,2)*noise_ay_);

  ekf_.Predict();

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    // Set H to the jacobian
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Update the Lidar Measurements
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
