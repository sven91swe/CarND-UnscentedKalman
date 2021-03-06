#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

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
  std_a_ = 1.5;
  //Guess according to part 31 in Lesson 7:Unscencted Kalman Filters. Pick half of the maximum I can expect of a bike.


  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.3;
  // Guess according to part 31 in Lesson 7:Unscencted Kalman Filters. Pick half of the maximum I can expect of a bike.


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

  n_x_ = 5;
  n_aug_ = 7;

  lambda_ = 3 - n_aug_;
}

UKF::~UKF() {}


/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::Init(MeasurementPackage meas_package) {
  cout << "Initilization with the first sensor reading." << endl;
  last_measurement_time = meas_package.timestamp_;

  x_aug_ = VectorXd(n_aug_);
  x_ = VectorXd(n_x_);

  P_ = MatrixXd(n_x_, n_x_);
  P_.fill(0.0);


  P_aug_ = MatrixXd(n_aug_, n_aug_);
  P_aug_.fill(0.0);
  P_aug_(5,5) = std_a_ * std_a_;
  P_aug_(6,6) = std_yawdd_ * std_yawdd_;

  weights_ = VectorXd(2*n_aug_+1);
  x_pred_ = VectorXd(n_x_);
  P_pred_ = MatrixXd(n_x_, n_x_);

  is_initialized_ = true;

  if(meas_package.sensor_type_ == meas_package.LASER) {
    x_(0) = meas_package.raw_measurements_(0);
    x_(1) = meas_package.raw_measurements_(1);
    x_(2) = 0;
    x_(3) = 0;
    x_(4) = 0;

    P_ << std_laspx_ * std_laspx_, 0, 0, 0, 0,
            0, std_laspy_ * std_laspy_, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;

  } else if(meas_package.sensor_type_ == meas_package.RADAR){
    //This scenario does not occur unless I disregard all Lidar measurements.
    cout << "Does this actually happen for any of the datasets?" << endl;

    double phi = meas_package.raw_measurements_(0);
    double rho = meas_package.raw_measurements_(1);
    double phi_dot = meas_package.raw_measurements_(2);

    x_(0) =  phi * cos(rho);
    x_(1) =  - phi * sin(rho);
    x_(2) =  phi_dot; //We know that there is some speed in that direction
    x_(3) =  rho; //Direction of known velocity.
    x_(4) = 0;

    //Unsure what to put here.
    P_.row(0)[0] = 0.5;
    P_.row(1)[1] = 0.5;
    P_.row(2)[2] = 0.5;
    P_.row(3)[3] = 0.5;
    P_.row(4)[4] = 1;

  }else{
    //Very strange scenario
    is_initialized_ = false;
  }

}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  if(!is_initialized_){
    //To initilize with the correct sensor type if one is disabled.
    if((meas_package.sensor_type_ == meas_package.RADAR) && use_radar_) {
      Init(meas_package);
    }else if((meas_package.sensor_type_ == meas_package.LASER) && use_laser_){
      Init(meas_package);
    }

  }else{

    double deltaT = (meas_package.timestamp_ - last_measurement_time)/pow(10.0,6);

    if((meas_package.sensor_type_ == meas_package.RADAR) && use_radar_){

      Prediction(deltaT);
      UpdateRadar(meas_package);

      last_measurement_time = meas_package.timestamp_;

    }else if((meas_package.sensor_type_ == meas_package.LASER) && use_laser_){

      Prediction(deltaT);

      UpdateLidar(meas_package);
      last_measurement_time = meas_package.timestamp_;

    }
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
//Parts of this code was taken from the examples and exercises in lesson 7.
void UKF::Prediction(double delta_t) {

  //create augmented mean vector
  x_aug_.head(5) = x_;
  x_aug_(5)=0;
  x_aug_(6)=0;

  P_aug_.topLeftCorner(5,5) = P_;

  //create sigma point matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.fill(0.0);

  //create square root matrix
  MatrixXd L = P_aug_.llt().matrixL();
  //cout << "x_aug_: " << x_aug_ << endl;

  //cout << "x_aug_ with additon: " << x_aug_ + sqrt(lambda_ + n_aug_) * L.col(3) << endl;

  //create augmented sigma points
  Xsig_pred_.col(0)  = tools.CTRVModelUpdate(delta_t, x_aug_);
  for (int i = 0; i< n_aug_; i++){
    Xsig_pred_.col(i+1)       = tools.CTRVModelUpdate(delta_t, x_aug_ + sqrt(lambda_ + n_aug_) * L.col(i));
    Xsig_pred_.col(i+1+n_aug_) = tools.CTRVModelUpdate(delta_t, x_aug_ - sqrt(lambda_ + n_aug_) * L.col(i));
  }


  // set weights
  weights_.fill(0.0);
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<2*n_aug_ + 1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }

  //predicted state mean
  x_pred_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_pred_ = x_pred_ + weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_pred_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_pred_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_pred_ = P_pred_ + weights_(i) * x_diff * x_diff.transpose() ;
  }

  //cout << x_pred_ << endl;

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
//Parts of this code was taken from the examples and exercises in lesson 7.
void UKF::UpdateLidar(MeasurementPackage meas_package) {

  int n_z = 2;


  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  Zsig.fill(0.0);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);

    // measurement model
    Zsig(0,i) = p_x;
    Zsig(1,i) = p_y;
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R.fill(0.0);
  R <<    std_laspx_*std_laspx_, 0,
          0, std_laspy_*std_laspy_;
  S = S + R;



  //Could probably be extracted to other function


  MatrixXd Tc = MatrixXd(n_x_, n_z);

  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_pred_;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

  //update state mean and covariance matrix
  x_ = x_pred_ + K * z_diff;
  P_ = P_pred_ - K*S*K.transpose();
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
//Parts of this code was taken from the examples and exercises in lesson 7.
void UKF::UpdateRadar(MeasurementPackage meas_package) {

  int n_z = 3;


  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  Zsig.fill(0.0);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y ,p_x);                                //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R.fill(0.0);
  R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
  S = S + R;



  //Could probably be extracted to other function


  MatrixXd Tc = MatrixXd(n_x_, n_z);

  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_pred_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;


  //update state mean and covariance matrix
  x_ = x_pred_ + K * z_diff;
  P_ = P_pred_ - K*S*K.transpose();
}