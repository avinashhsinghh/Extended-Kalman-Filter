#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

// KF and EKF is based on Wikipeida.

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
    x_ = F_*x_;
    P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    VectorXd y = z - H_ * x_;
    MatrixXd S = R_ + H_ * P_ * H_.transpose();
    MatrixXd K = P_*H_.transpose()*S.inverse();
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size,x_size);
    x_ = x_ + K * y;
    P_ = (I-K*H_)*P_*(I-K*H_).transpose() + K*R_*K.transpose(); // This is the algorithm on Wikipedia
    // P_ = (I-K*H_)*P_; // This is algorithm instructed in the Udacity lecture.

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

    // Convert to Cartesian space.
    double rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
    double theta = atan2(x_(1), x_(0));
    double rho_dot = (x_(0)*x_(2)+x_(1)*x_(3))/rho;

    VectorXd h = VectorXd(3);
    h << rho,theta,rho_dot;
    VectorXd y = z - h;
    while (y(1)>M_PI || y(1) < -M_PI) {
        if (y(1) > M_PI) {
            y(1) -= 2 * M_PI;
        }
        if (y(1) < -M_PI) {
            y(1) += 2 * M_PI;
        }
    }
    Hj_ = tools.CalculateJacobian(x_);
    MatrixXd S = R_ + Hj_ * P_ * Hj_.transpose();
    MatrixXd K = P_*Hj_.transpose()*S.inverse();
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size,x_size);
    x_ = x_ + K * y;
    P_ = (I-K*Hj_)*P_;

}
