#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
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

  H_laser_<<1,0,0,0,
        0,1,0,0;

  ekf_.P_ = MatrixXd(4,4);
  ekf_.P_ << 1,0,0,0,
            0,1,0,0,
            0,0,1000,0,
            0,0,0,1000;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

    // first measurement
    previous_timestamp_ = measurement_pack.timestamp_;
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
        cout<<"Radar Measurement Initializing"<<endl;
        double rho = measurement_pack.raw_measurements_[0];
        double theta = measurement_pack.raw_measurements_[1];
        double rho_dot = measurement_pack.raw_measurements_[2];

        double x = rho*cos(theta);
        double y = rho*sin(theta);
        double x_dot = rho_dot*cos(theta);
        double y_dot = rho_dot*sin(theta);

        ekf_.x_ << x,y,x_dot,y_dot;


    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      cout<<"Laser Measurement Initializing"<<endl;
      ekf_.x_ << measurement_pack.raw_measurements_[0],measurement_pack.raw_measurements_[1],0,0;

    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  double dt = (measurement_pack.timestamp_-previous_timestamp_)/1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  double noise_ax = 9.0;
  double noise_ay = 9.0;
  double dt2 = dt*dt;
  double dt3 = dt2*dt;
  double dt4 = dt3*dt;
  MatrixXd Q = MatrixXd(4,4);
  Q << (dt4/4)*noise_ax,0,(dt3/2)*noise_ax,0,
        0,(dt4/4)*noise_ay,0,(dt3/2)*noise_ay,
        (dt3/2)*noise_ax,0,dt2*noise_ax,0,
        0,(dt3/2)*noise_ay,0,dt2*noise_ay;

  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1,0,dt,0,
            0,1,0,dt,
            0,0,1,0,
            0,0,0,1;

  ekf_.H_ = H_laser_;


  ekf_.Q_ = Q;
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/


  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    // Radar updates
  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
