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
  //measurement matrix
  H_laser_<< 1, 0, 0, 0,
	  0, 1, 0, 0;

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
   //set the acceleration noise components
  noise_ax = 5;
  noise_ay = 5;

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
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

	//state covariance matrix P
	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1000, 0,
			0, 0, 0, 1000;		
	//the initial transition matrix F_
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 1, 0,
			0, 1, 0, 1,
			0, 0, 1, 0,
			0, 0, 0, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

		previous_timestamp_ = measurement_pack.timestamp_;

        //Convert radar from polar to cartesian coordinates and initialize state.
        float rho = measurement_pack.raw_measurements_(0);
        float phi =  measurement_pack.raw_measurements_(1);
        float rhodot =  measurement_pack.raw_measurements_(2);
        ekf_.x_(0) = rho*cos(phi);
        ekf_.x_(1) = rho*sin(phi);
        ekf_.x_(2) = 0; 
        ekf_.x_(3) = 0; 
		
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {

	  	ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

		if (fabs(ekf_.x_(0)) < 0.0001)
		{
			fabs(ekf_.x_(0) = 0.01;
		}
		if (fabs(ekf_.x_(1)) < 0.0001)
		{
			fabs(ekf_.x_(1) = 0.01;
		}
    }

    // done initializing, no need to predict or update
	previous_timestamp_ = measurement_pack.timestamp_;
	is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  //For this project, however, we do not need to use the f function or Fj for Radar
  //Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			   0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			   dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			   0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

	float px = ekf_.x_[0];
    float py = ekf_.x_[1];
    float vx = ekf_.x_[2];
    float vy = ekf_.x_[3];

	float rho;
    float phi;
    float rhodot;

    rho = sqrt(px*px + py*py);

    // Avoid Divide by Zero throughout the Implementation
	if(fabs(px) < 0.0001 or fabs(py) < 0.0001)
	{
        if(fabs(px) < 0.0001)
		{
          px = 0.0001; 
        }

        if(fabs(py) < 0.0001)
		{
          py = 0.0001;
        }
        
        rho = sqrt(px*px + py*py);
		//set phi  and radial velocity to zero
        phi = 0;	
        rhodot = 0;	
  
    } else {
        rho = sqrt(px*px + py*py);
        phi = atan2(py,px);
        rhodot = (px*vx + py*vy) /rho;
	}
    
	// measurements h(x)
	ekf_.hx_ << rho, phi, rhodot;
	ekf_.R_ = R_radar_;

	ekf_.Hj_ = tools.CalculateJacobian(ekf_.x_); //H_jacob is calculated
	ekf_.H_ = ekf_.Hj_; //H-jacobian is passed

    // Radar updates
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
	ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
