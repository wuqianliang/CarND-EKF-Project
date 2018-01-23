#include "kalman_filter.h"
#include <math.h>
using Eigen::MatrixXd;
using Eigen::VectorXd;

const float Float_2PI = 2 * M_PI;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
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
    * update the state by using Extended Kalman Filter equations
  */

    float   px      = x_[0];
    float   py      = x_[1];
    float   vx      = x_[2];
    float   vy      = x_[3];

    float   rho;
    float   phi;
    float   rhodot;

    rho     = sqrt( px * px + py * py );
    /* Avoid Divide by Zero throughout the Implementation */
    if( rho < 0.000001)
      rho = 0.000001;

    phi     = atan2( py, px );
    rhodot  = (px * vx + py * vy) / rho;

    /* Measurements h(x) */
    VectorXd z_pred = VectorXd( 3 );
    z_pred  << rho, phi, rhodot;

    VectorXd y = z - z_pred;

    // Normalize angle phi in the y vector between -pi and pi.
    bool between_minus_pi_and_pi = false;
    
    while (between_minus_pi_and_pi == false) {
      if (y(1) > M_PI) {
        y(1) = y(1) - Float_2PI; //-2pi
      }
      else if (y(1) < -M_PI) {
        y(1) = y(1) + Float_2PI;//2pi
      } 
      else {
        between_minus_pi_and_pi = true;
      }
    }

    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;

}
