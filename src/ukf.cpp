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
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.8;
  
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

  // Initialize time
  time_us_ = 0.0;

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
    // Update boolean after initializing state vector and process covariance matrix
    is_initialized_ = true; 
    return;
  }

  // Prediction Function call to generate and predict sigma points
  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;
  
  Prediction(delta_t);

  // Update state and covariance vector based on sensor measurement type
  if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER && use_laser_) 
  {
    UpdateLidar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR && use_radar_)
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

  /* Step 1 : Generate Sigma points for prediction */
  // Create augmented state vector
  VectorXd x_aug_ = VectorXd(n_aug_).setZero();
  // Create augmented covariance matrix
  MatrixXd P_aug_ = MatrixXd(n_aug_,n_aug_).setZero();
  // Create augmented sigma point vector
  MatrixXd x_sig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1).setZero();

  // Create augmented mean state
  x_aug_.head(5) = x_;
  x_aug_(5) = 0.0;
  x_aug_(6) = 0.0;

  // Create augmented covariance matrix
  P_aug_.topLeftCorner(5,5) = P_;
  P_aug_(5,5) = powSquare(std_a_);
  P_aug_(6,6) = powSquare(std_yawdd_);

  // Create the square root matrix
  MatrixXd A = P_aug_.llt().matrixL();

  // Generate augmented sigma points
  x_sig_aug_.col(0) = x_aug_;
  for (int i = 0; i < n_aug_; ++i) 
  {
    x_sig_aug_.col(i+1) = x_aug_ + sqrt(lambda_ + n_aug_) * A.col(i);
    x_sig_aug_.col(i+1+n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * A.col(i);
  }

  /* Step 2 : Sigma point prediction */
  for (int i = 0; i < 2 * n_aug_+ 1; ++i) 
  {
    // extract values for better readability
    double px = x_sig_aug_(0,i);
    double py = x_sig_aug_(1,i);
    double v = x_sig_aug_(2,i);
    double yaw = x_sig_aug_(3,i);
    double yawd = x_sig_aug_(4,i);
    double nu_a = x_sig_aug_(5,i);
    double nu_yawdd = x_sig_aug_(6,i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = px + v / yawd * ( sin(yaw + yawd * delta_t) - sin(yaw) );
        py_p = py + v / yawd * ( cos(yaw) - cos(yaw + yawd * delta_t) );
    } else {
        px_p = px + v * delta_t * cos(yaw);
        py_p = py + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5 * nu_a * powSquare(delta_t) * cos(yaw);
    py_p = py_p + 0.5 * nu_a * powSquare(delta_t) * sin(yaw);
    v_p = v_p + nu_a * delta_t;

    yaw_p = yaw_p + 0.5 * nu_yawdd * powSquare(delta_t);
    yawd_p = yawd_p + nu_yawdd * delta_t;

    // write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  /* Step 3 : Predicting Mean and Covariance */
  // Predict state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) 
  {  // iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  // Predict covariance
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) 
  { // iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  // Set measurement dimension
  int n_z_ = 2;

  // create matrix for sigma points in measurement space
  MatrixXd zSig_ = MatrixXd(n_z_, 2 * n_aug_ + 1).setZero();

  // Mean predicted measurement
  VectorXd z_pred_ = VectorXd(n_z_).setZero();

  // Measurement covariance matrix
  MatrixXd S_ = MatrixXd(n_z_, n_z_).setZero();
  
  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_).setZero();

  // Transform sigma points to measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) 
  {
    zSig_(0,i) = Xsig_pred_(0,i);
    zSig_(1,i) = Xsig_pred_(1,i);
  }

  // mean predicted measurement and update
  z_pred_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) 
  {
    z_pred_ = z_pred_ + weights_(i) * zSig_.col(i);
  }

  // innovation covariance matrix S and cross correlation matrix
  S_.fill(0.0);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) 
  {  // 2n+1 simga points
    // residual
    VectorXd z_diff = zSig_.col(i) - z_pred_;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S_ = S_ + weights_(i) * z_diff * z_diff.transpose();

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    // calculate cross correlation matrix
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z_, n_z_);
  R <<  powSquare(std_laspx_), 0,
        0, powSquare(std_laspy_);
        
  S_ = S_ + R;

  // Kalman gain K;
  MatrixXd K = Tc * S_.inverse();

  // residual
  VectorXd z_ = meas_package.raw_measurements_;
  VectorXd z_diff = z_ - z_pred_;

  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S_ * K.transpose();

  // Calculate NIS for LIDAR
  NIS_laser_ = z_diff.transpose() * S_.inverse() * z_diff;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  // Set measurement dimension
  int n_z_ = 3;

  // create matrix for sigma points in measurement space
  MatrixXd zSig_ = MatrixXd(n_z_, 2 * n_aug_ + 1).setZero();

  // Mean predicted measurement
  VectorXd z_pred_ = VectorXd(n_z_).setZero();

  // Measurement covariance matrix
  MatrixXd S_ = MatrixXd(n_z_, n_z_).setZero();
  
  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_).setZero();

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // extract values for better readability
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double vx = cos(yaw) * v;
    double vy = sin(yaw) * v;

    // measurement model
    zSig_(0,i) = sqrt(px * px + py * py);                       // r
    zSig_(1,i) = atan2(py,px);                                  // phi
    zSig_(2,i) = (px * vx + py * vy) / sqrt(px * px + py * py); // r_dot
  }

  // mean predicted measurement and update
  z_pred_.fill(0.0);
  for (int i=0; i < 2 * n_aug_ + 1; ++i) 
  {
    z_pred_ = z_pred_ + weights_(i) * zSig_.col(i);
  }

  // innovation covariance matrix S and cross correlation matrix
  S_.fill(0.0);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) 
  {  // 2n+1 simga points
    // residual
    VectorXd z_diff = zSig_.col(i) - z_pred_;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S_ = S_ + weights_(i) * z_diff * z_diff.transpose();

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    // calculate cross correlation matrix
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z_, n_z_);
  R <<  powSquare(std_radr_), 0, 0,
        0, powSquare(std_radphi_), 0,
        0, 0,powSquare(std_radrd_);
  S_ = S_ + R;

  // Kalman gain K;
  MatrixXd K = Tc * S_.inverse();

  // residual
  VectorXd z_ = meas_package.raw_measurements_;
  VectorXd z_diff = z_ - z_pred_;

  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S_ * K.transpose();

  // Calculate NIS radar
  NIS_radar_ = z_diff.transpose() * S_.inverse() * z_diff;
}