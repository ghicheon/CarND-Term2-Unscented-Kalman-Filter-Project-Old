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
  std_a_ = 0.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2;
  
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

  //set state dimension
  n_x_ = 5;

  //set augmented dimension
  n_aug_ = 7;


  //define spreading parameter
  lambda_ = 3 - n_aug_;

  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  weights_ = VectorXd(2*n_aug_+1);



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
    if( use_laser_ == false && meas_package.sensor_type_ == MeasurementPackage::LASER)
        return ;

    if( use_radar_ == false && meas_package.sensor_type_ == MeasurementPackage::RADAR)
        return ;
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  if (!is_initialized_) {
      // first measurement
      cout << "UKF: " << endl;
     
   //   x_  << 1,1,0.1,0.1,0.1;  //XXX
     
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
          x_(4) = 0 ;
          printf("XXXXXXXXXXXXX ProcessMeasurement init radar\n");
      }
      else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        x_(0) =  meas_package.raw_measurements_[0];
        x_(1) =  meas_package.raw_measurements_[1];
        x_(2) = 5.199937e+00 ;
        x_(3) = 0 ;
        x_(4) = 0 ;
          printf("XXXXXXXXXXXXX ProcessMeasurement init laser\n");
     
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

  /////////////////////////////////

 
  int i=0;

printf("Predict x1");
  weights_(0)= lambda_ / (lambda_+n_aug_);
  for(i=1;i< (2*n_aug_+1) ; i++) weights_(i) = 1/(2*(lambda_+n_aug_)) ;

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

  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  
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
printf("Predict x2");
  
  for(int i=0; i<7;i++)
  {
      xxx.col(i) = x_aug;
  }
    
  Xsig_aug.block<7,7>(0,1) = xxx  + sqrt(lambda_ +n_aug_) * L;
  
  Xsig_aug.block<7,7>(0,8) = xxx  - sqrt(lambda_ +n_aug_) * L;

  //PPP 2 /////////////////////////////////////////////////////////////////////////////// 


printf("Predict x3");
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
       for(int i=0; i < 2*n_aug_+1;i++)
       {
           x = Xsig_aug(0,i);
           y = Xsig_aug(1,i);
           v =Xsig_aug(2,i);
           s = Xsig_aug(3,i);
           s_ = Xsig_aug(4,i);
           u = Xsig_aug(5,i);
           u__ = Xsig_aug(6,i);
           
printf("Predict x4");
           if( s_ == 0)
           {
               Xsig_pred_(0,i)=x + v*cos(s)*t   + 1/2.0 * t*t * cos(s) * u;
               Xsig_pred_(1,i)=y + v*sin(s)*t + 1/2.0 * t*t * sin(s) * u;
           }
           else
           {
               Xsig_pred_(0,i) = x + v/s_ *(sin(s + s_*t) - sin(s)) + 1/2.0 * t*t * cos(s)*u;
               Xsig_pred_(1,i) = y + v/s_ *(-cos(s+s_*t) + cos(s)) + 1/2.0 * t*t * sin(s)*u;
           }
           Xsig_pred_(2,i)= v + 0 + t*u;
           Xsig_pred_(3,i)= s + s_ * t + 1/2.0 * t*t * u__;
           Xsig_pred_(4,i)= s_ + 0 + t * u__;
       
       }
  }
  

  //PPP 3 ////////////////////////////////////////////////////////////////
  //predict state mean
  //predict state covariance matrix
  
printf("Predict x5");
  VectorXd x_temp = VectorXd(n_x_);
  
  x_temp.fill(0);
  for(i=0;i < (n_aug_*2+1) ; i++)
  {
      x_temp += weights_(i) * Xsig_pred_.col(i);
  }

printf("Predict x6");
  MatrixXd P_temp = MatrixXd(5, 5);
printf("Predict x60");
  P_temp.fill(0);
printf("Predict x61");
  for(i=0;i<(n_aug_*2+1) ; i++)
  {
      VectorXd diff = Xsig_pred_.col(i) - x_temp ;
printf("Predict x62");
      while( diff(3) < -3.14) diff(3) += 2*3.14;
printf("Predict x63");
      while( diff(3) > 3.14)  diff(3) -= 2*3.14;
printf("Predict x64");
      
      P_temp += weights_(i) * diff * diff.transpose();
  }

printf("Predict x7");
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

 MatrixXd  H_ = MatrixXd(2, 5);

  R_ << 0.0225, 0,
        0, 0.0225;

   H_     << 1,0,0,0,0,
             0,1,0,0,0;
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
 MatrixXd z = meas_package.raw_measurements_;

    //UpdateRadar(meas_package.raw_measurements_);

  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */


  //26. XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  int n_x_ = 5;

  int n_aug_ = 7;

  int n_z = 3;

  double lambda_ = 3 - n_aug_;

  //std_radr_ = 0.3;
  //std_radphi_ = 0.03;
  //std_radrd_ = 0.3;

  VectorXd weights_ = VectorXd(2*n_aug_+1);
 
  int i=0;
  weights_(0)= lambda_ / (lambda_+n_aug_);
  for(i=1;i< (2*n_aug_+1) ; i++) weights_(i) = 1/(2*(lambda_+n_aug_)) ;


 //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  //transform sigma points into measurement space
  for(int i=0; i < 2*n_aug_+1 ; i++)
  {
      VectorXd x = Xsig_pred_.col(i);
    float px = x(0);
    float py = x(1);
    float v = x(2);
    float psi = x(3);
    float psi_ = x(4);
    
    float rho = sqrt(px*px + py*py);
    float pi = atan2(py,px);
    float rho_ = (px*cos(psi)*v + py*sin(psi)*v)/sqrt(px*px + py*py) ;
    
    Zsig.col(i) << rho,pi,rho_;
  }
  
  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for(int i=0; i < 2*n_aug_+1; i++)
  {
     z_pred += weights_(i) * Zsig.col(i);
  }
  
  //calculate innovation covariance matrix S
  S.fill(0.0);
  MatrixXd R  = MatrixXd(3,3);
  R.fill(0.0);
  R(0,0) = std_radr_*std_radr_;
  R(1,1) = std_radphi_*std_radphi_;
  R(2,2) = std_radrd_*std_radrd_;
  
  
  
  for(int i=0; i < 2*n_aug_+1 ; i++)
  {
      VectorXd diff = Zsig.col(i) - z_pred;
      S  += weights_(i) * diff * diff.transpose() ;
  }
  S += R;


  //30. XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  //calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for(int i=0; i< n_aug_*2+1 ; i++)
  {
      VectorXd xdiff = Xsig_pred_.col(i) - x_;
      while( xdiff(1) > 3.14)   xdiff(1) -= 2*3.14;
      while( xdiff(1) < -3.14)   xdiff(1) += 2*3.14;
      

      VectorXd zdiff = Zsig.col(i) - z_pred;
      while( zdiff(1) > 3.14)   zdiff(1) -= 2*3.14;
      while( zdiff(1) > 3.14)   zdiff(1) -= 2*3.14;      
      
      //Tc += weights_(i) * (Xsig_pred_.col(i) - x) *
      //              (Zsig.col(i) - z_pred).transpose();
      Tc += weights_(i) * xdiff * zdiff.transpose();
  }
  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  //update state mean and covariance matrix
  
  x_  = x_ + K*(z - z_pred);
  P_ = P_ - K*S*K.transpose();
}

//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX






#if 0


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
