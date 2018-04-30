#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
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

  P_ <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
          -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
           0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
          -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
          -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  is_initialized_ = false;
printf("XXXXXXXXXXXXX0\n");
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
printf("XXXXXXXXXXXXX ProcessMeasurement\n");
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  if (!is_initialized_) {
      // first measurement
      cout << "UKF: " << endl;
     
      x_  << 1,1,0.1,0.1,0.1;  //XXX
     
      if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
          //Convert radar from polar to cartesian coordinates and initialize state.
          float rho     = meas_package.raw_measurements_[0];
          float phi     = meas_package.raw_measurements_[1];
          float rho_dot = meas_package.raw_measurements_[2];
     
          float px = rho*cos(phi); 
          float py = rho*sin(phi);
     
          x_(0) = px ;
          x_(1) = py ;
          x_(2) = 0.1 ;
          x_(3) = 0.1 ;
      }
      else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        x_(0) =  meas_package.raw_measurements_[0];
        x_(1) =  meas_package.raw_measurements_[1];
        x_(2) = 5.199937e+00 ;
        x_(3) = 0 ;
     
      }
     
      time_us_ = meas_package.timestamp_;
     
      // done initializing, no need to predict or update
      is_initialized_ = true;
printf("XXXXXXXXXXXXX1\n");
      return;
  }
printf("XXXXXXXXXXXXX1.5\n");

  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;     //dt - expressed in seconds

  time_us_ = meas_package.timestamp_;
printf("XXXXXXXXXXXXX1.6\n");

  /*
   * Predict
   */
  Prediction(dt);
printf("XXXXXXXXXXXXX2\n");

  /*
   * Update
   */
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

    //XXXXXXXX   UpdateRadar(meas_package);
    // Radar updates
  } else {
    // Laser updates
    UpdateLidar(meas_package);
  }
printf("XXXXXXXXXXXXX3\n");

  // print the output
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;


  //define spreading parameter
  double lambda = 3 - n_aug;

  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

  /////////////////////////////////

  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

  VectorXd weights = VectorXd(2*n_aug+1);

 
  int i=0;

  weights(0)= lambda / (lambda+n_aug);
  for(i=1;i< (2*n_aug+1) ; i++) weights(i) = 1/(2*(lambda+n_aug)) ;

  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */


  //PPP 1 /////////////////////////////////////////////////////////////////////////////// 
  //create augmented mean state
  //create augmented covariance matrix
  //create square root matrix
  //create augmented sigma points
  
  x_aug.head(5)= x_;
  x_aug(5)=0;
  x_aug(6)=0;
  
  P_aug.fill(0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;
  
  MatrixXd L = P_aug.llt().matrixL();
  Xsig_aug.col(0) = x_aug;
  
  MatrixXd xxx = MatrixXd(7,7);
  
  for(int i=0; i<7;i++)
  {
      xxx.col(i) = x_aug;
  }
    
  Xsig_aug.block<7,7>(0,1) = xxx  + sqrt(lambda +n_aug) * L;
  
  Xsig_aug.block<7,7>(0,8) = xxx  - sqrt(lambda +n_aug) * L;

  //PPP 2 /////////////////////////////////////////////////////////////////////////////// 


  //predict sigma points
  //avoid division by zero
  //write predicted sigma points into right column
  

  {
       float x;  //px
       float y;  //py
       float v;
       float s;  //psi
       float s_; //psi dot
       float u;  //myu a
       float u__;//myu psi dot dot
       float t = delta_t;
       for(int i=0; i < 2*n_aug+1;i++)
       {
           x = Xsig_aug(0,i);
           y = Xsig_aug(1,i);
           v =Xsig_aug(2,i);
           s = Xsig_aug(3,i);
           s_ = Xsig_aug(4,i);
           u = Xsig_aug(5,i);
           u__ = Xsig_aug(6,i);
           
           if( s_ == 0)
           {
               Xsig_pred(0,i)=x + v*cos(s)*t   + 1/2.0 * t*t * cos(s) * u;
               Xsig_pred(1,i)=y + v*sin(s)*t + 1/2.0 * t*t * sin(s) * u;
           }
           else
           {
               Xsig_pred(0,i) = x + v/s_ *(sin(s + s_*t) - sin(s)) + 1/2.0 * t*t * cos(s)*u;
               Xsig_pred(1,i) = y + v/s_ *(-cos(s+s_*t) + cos(s)) + 1/2.0 * t*t * sin(s)*u;
           }
           Xsig_pred(2,i)= v + 0 + t*u;
           Xsig_pred(3,i)= s + s_ * t + 1/2.0 * t*t * u__;
           Xsig_pred(4,i)= s_ + 0 + t * u__;
       
       }
  }
  

  //PPP 3 ////////////////////////////////////////////////////////////////
  //predict state mean
  //predict state covariance matrix
  
  VectorXd x_temp = VectorXd(n_x);
  
  x_temp.fill(0);
  for(i=0;i < (n_aug*2+1) ; i++)
  {
      x_temp += weights(i) * Xsig_pred.col(i);
  }

  VectorXd P_temp = MatrixXd(5, 5);
  P_temp.fill(0);
  for(i=0;i<(n_aug*2+1) ; i++)
  {
      VectorXd diff = Xsig_pred.col(i) - x_temp ;
      while( diff(3) < -3.14) diff(3) += 2*3.14;
      while( diff(3) > 3.14)  diff(3) -= 2*3.14;
      
      P_temp += weights(i) * diff * diff.transpose();
  }

  x_ = x_temp;
  P_ = P_temp;
  
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
 MatrixXd z = meas_package.raw_measurements_;

 MatrixXd  R_ = MatrixXd(2, 2);

 MatrixXd  H_ = MatrixXd(2, 4);

  R_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

   H_     << 1,0,0,0,
             0,1,0,0;
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

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    //UpdateRadar(meas_package.raw_measurements_);

  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}

//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX






#if 0

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


  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

   H_laser_  << 1,0,0,0,
             0,1,0,0;

   ekf_.P_ = MatrixXd(4, 4);
   ekf_.P_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;
   
   ekf_.F_ =  MatrixXd(4, 4);
   ekf_.F_  << 1, 0, 1, 0,
               0, 1, 0, 1,
               0, 0, 1, 0,
               0, 0, 0, 1;


   //noise_ax = 9;
   //noise_ay = 9;



  VectorXd xx = VectorXd(5);
  xx << 1,1,0.1,0.1,0.1;


  MatrixXd PP = MatrixXd(4, 4);
   PP << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;
  MatrixXd FF =  MatrixXd(4, 4);
  FF << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
  MatrixXd HH = MatrixXd(2, 4);
  MatrixXd RR = MatrixXd(2, 2);
  MatrixXd QQ = MatrixXd(4, 4);
  ekf_.Init( xx, PP, FF,HH,RR,QQ);



//
//  MatrixXd PP = MatrixXd(4, 4);
//  PP << 1, 0, 0, 0,
//        0, 1, 0, 0,
//        0, 0, 1000, 0,
//        0, 0, 0, 1000;
//
//  MatrixXd FF = MatrixXd(4, 4);
//  FF << 1, 0, 1, 0,
//        0, 1, 0, 1,
//        0, 0, 1, 0,
//        0, 0, 0, 1;
//
//
//  MatrixXd HH = MatrixXd(2, 4);
//  HH  << 1, 0, 0, 0,
//         0, 1, 0, 0;
//
//  MatrixXd RR = MatrixXd(2, 2);
//  RR << 0.0225, 0,
//        0, 0.0225;
//
//
//
//  MatrixXd QQ = MatrixXd(4, 4);
//  ekf_.Init( xx, PP, FF,HH,RR,QQ);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}



void FusionEKF::ProcessMeasurement(const MeasurementPackage &meas_package) {
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
    //ekf_.x_ = VectorXd(4);
    //ekf_.x_ << 1, 1, 1, 1;

  ekf_.x_  = VectorXd(4);
  ekf_.x_  << 1,1,0.1,0.1; 

    previous_timestamp_ = meas_package.timestamp_;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        /**
        Convert radar from polar to cartesian coordinates and initialize state.
        */
        float rho     = meas_package.raw_measurements_[0];
        float phi      = meas_package.raw_measurements_[1];
        float rho_dot = meas_package.raw_measurements_[2];

        float px = rho*cos(phi); 
        float py = rho*sin(phi);

        ekf_.x_(0) = px ;
        ekf_.x_(1) = py ;
        ekf_.x_(2) = 0.1 ;
        ekf_.x_(3) = 0.1 ;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_(0) =  meas_package.raw_measurements_[0];
      ekf_.x_(1) =  meas_package.raw_measurements_[1];
      ekf_.x_(2) = 5.199937e+00 ;
      ekf_.x_(3) = 0 ;

    }

    previous_timestamp_ = meas_package.timestamp_;

    // done initializing, no need to predict or update
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
  float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;     //dt - expressed in seconds
  previous_timestamp_ = meas_package.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  //Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  int noise_ax = 9;
  int noise_ay = 9;

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

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

    ekf_.R_ = R_radar_;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(meas_package.raw_measurements_);
    // Radar updates
  } else {
    // Laser updates

    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(meas_package.raw_measurements_);
  }

  // print the output
 cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
#include "kalman_filter.h"

#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
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






//cartition -> polar
VectorXd h( VectorXd &x) 
{
    VectorXd v = VectorXd(3);

    float px = x(0);
    float py = x(1);
    float vx = x(2);
    float vy = x(3);

    v(0) = sqrt(px*px+py*py);

    v(1) = 0.00001;
    if( px != 0 )
        v(1) = atan2(py,px);

    v(2) = 0.00001;
    if((px*px+py*py) != 0)
        v(2) = (px*vx + py*vy)/sqrt(px*px+py*py);

    while( v(1) < -3.14 ) v(1) += 3.14;
    
    while( v(1) > 3.14 ) v(1) -= 3.14;

    return v;
}


void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  Tools tools;

  VectorXd z_pred = h(x_);
  VectorXd y = z - z_pred;

 y(1) = atan2(sin(y(1)), cos(y(1)));//nomalize

  while( y(1) < -3.14 )
  {
      printf("YYYYYYYYYYYYYYYYYYYYYYY-   %f\n", y(1));
      y(1) += 3.14;
  }
  
  while( y(1) > 3.14 )
  {
      printf("YYYYYYYYYYYYYYYYYYYYYYY+        %f\n", y(1));
      y(1) -= 3.14;
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
#endif
