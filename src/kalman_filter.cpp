#include "kalman_filter.h"
#include "iostream"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in; // object state
  P_ = P_in; // object covariance matrix
  F_ = F_in; // state transition matrix
  H_ = H_in; // measurement matrix
  R_ = R_in; // measurement covariance matrix
  Q_ = Q_in; // process covariance matrix

  //VectorXd u;	// external motion
  //MatrixXd I; // Identity matrix
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_ ; // x = Fx+u Where u<<0,0,0,0 because noise mean is 0;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;   // error
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
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
   //map predicated state x_ into RADAR measurement space: rho, theta, rho_dot
   //Formula is from Lecture L5.14
   float px = x_(0);
   float py = x_(1);
   float vx = x_(2);
   float vy = x_(3);

   // Cartesian to polar   
   double rho = sqrt(px*px + py*py);
   //double theta = atan(py / px);
   double theta = atan2(py, px);  // In C++, atan2() returns values between -pi and pi
   double rho_dot = (px*vx + py*vy) / rho;
   VectorXd z_pred = VectorXd(3); // h(x_)
   z_pred << rho, theta, rho_dot;
   
   // Calculations are essentially the same to the Update function
   VectorXd y = z - z_pred;  //error

   /* //Lecture L5.20
     You'll need to make sure to normalize ϕ in the y vector so that its angle is between −pi and pi; 
     in other words, add or subtract 2pi from ϕ until it is between −pi and pi. */
  const double pi = 3.14;  // 3.14159265358979323846
  if (y(1) < -pi) {
    std::cout << "---------------------------------------- Theta is < -3.14.  ϕ = " << y(1);
    y(1) = y(1) + 2*pi;
    std::cout << " ---------------------------------------- Normalized Theta ϕ = " << y(1) << std::endl;

  }
  else if (y(1) > pi) {
    std::cout << "---------------------------------------- Theta is < -3.14.  ϕ = " << y(1);
    y(1) = y(1) - 2*pi;
    std::cout << " ---------------------------------------- Normalized Theta ϕ = " << y(1) << std::endl;
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
