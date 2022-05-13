#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 4;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 3;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  // State dimension
  n_x_ = 5;

  // Augmented vector dimension
  n_aug_ = 7;

  // Design parameter for UKF
  lambda_ = 3 - n_x_;

  // Initialize process covariance matrix
  P_.setIdentity(5,5);

  // Process measurement boolean
  is_initialized_ = false;

  // NIS initialization
  NIS_radar_ = 0.0;
  NIS_laser_ = 0.0;

  // Place holder for Sigma predicted points
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // Place holder for weights vector
  weights_ = VectorXd(2 * n_aug_ + 1);

  // Set the weights vector
  weights_.fill(0.5 / (n_aug_+lambda_));
  weights_(0) = lambda_ / (n_aug_+lambda_);

}

UKF::~UKF() {}

double UKF::powSquare(double value) {

  // Return the square of the value. Mimicing the power function
  return (value * value);
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  if (!is_initialized_)
  {
    double px, py, vel = 0;
    // Initialize the state vector and process covairance matrix from Raw measurements of the sensor
    if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER)
    {
      // Read raw measurements from the lidar sensor
      px = meas_package.raw_measurements_(0);
      py = meas_package.raw_measurements_(1);

      // Set the state vector
      x_ << px, py, 0, 0, 0;

      // Set the process covariance vector
      P_ << powSquare(std_laspx_), 0, 0, 0, 0,
            0, powSquare(std_laspy_), 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR)
    {
      // Read raw measurements from radar sensor
      double rho = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      double rho_dot = meas_package.raw_measurements_(2);

      px = rho * cos(phi);
      py = rho * sin(phi);
      vel = rho_dot;

      // Set the state vector
      x_ << px, py, vel, rho, rho_dot;

      // Set the process covariance vector
      P_ << powSquare(std_radr_), 0, 0, 0, 0,
            0, powSquare(std_radr_), 0, 0, 0,
            0, 0, powSquare(std_radrd_), 0, 0,
            0, 0, 0, powSquare(std_radphi_), 0,
            0, 0, 0, 0, powSquare(std_radphi_);
    }
    time_us_ = meas_package.timestamp_;
    // Update boolean after initializing state vector and process covariance matrix
    is_initialized_ = true; 
    
  }

  // Prediction Function call to generate and predict sigma points
  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  Prediction(delta_t);

  // Update state and covariance vector based on sensor measurement type
  if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER) 
  {
    UpdateLidar(meas_package);
  }
  else
  {
    UpdateRadar(meas_package);
  }
  
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
}