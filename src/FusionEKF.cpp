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
  F_ = MatrixXd(4, 4);
  Q_ = MatrixXd(4, 4);

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
  H_laser_ << 1, 0, 0, 0,
	      0, 1, 0, 0;

  Hj_ << 0, 0, 0, 0,
	 0, 0, 0, 0,
	 0, 0, 0, 0;
  
  F_ << 1, 0, 0, 0,
	0, 1, 0, 0,
	0, 0, 1, 0,
	0, 0, 0, 1;

  noise_ax = 9.0f;
  noise_ay = 9.0f;
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

    VectorXd state_init(4);
    MatrixXd P_init(4, 4);
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */

      float ro,theta,px,py;
      
      ro =  measurement_pack.raw_measurements_[0];
      theta = measurement_pack.raw_measurements_[1];

      px = ro * cos(theta);
      py = ro * sin(theta);

      //set the state with the initial location and zero velocity
      state_init << px, py, 0, 0;
      P_init << 10, 0, 0, 0,
	        0, 10, 0, 0,
	   	0, 0, 100, 0,
	   	0, 0, 0, 100;
    }

    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      
      //set the state with the initial location and zero velocity
      state_init << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      P_init << 10, 0, 0, 0,
	   	0, 10, 0, 0,
	   	0, 0, 100, 0,
	   	0, 0, 0, 100;
    }

    // done initializing, no need to predict or update
    ekf_.Init(state_init,P_init);
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

  cout<<"-------------------------------------------------------------------"<<endl;
  cout<<"Time Step = "<<measurement_pack.timestamp_/1000000.0<<endl;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  //Modify the F matrix so that the time is integrated
  F_(0, 2) = dt;
  F_(1, 3) = dt;

  //set the process covariance matrix Q

  Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
	     0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
	     dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
	     0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
  
  ekf_.Predict(F_,Q_);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    cout<<"Radar Update"<<endl;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_,R_radar_);
  } else {
    // Laser updates
    cout<<"Laser Update"<<endl;
    ekf_.Update(measurement_pack.raw_measurements_,H_laser_,R_laser_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
