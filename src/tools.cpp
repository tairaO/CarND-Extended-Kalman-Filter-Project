#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
    * Calculate the RMSE here.
  */

VectorXd rmse_(4);
rmse_ << 0, 0, 0, 0;
unsigned int est_num_ =  estimations.size();

  if(est_num_ != ground_truth.size() || est_num_==0) {
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse_;
  }

  // accumulate residuals
  for(unsigned int i=0; i < est_num_; ++i) {
    // calculate squared error
    VectorXd residual_ = estimations[i] - ground_truth[i];
    residual_ = residual_.array()*residual_.array();
    rmse_ += residual_;
  }

  // calculate the mean
  rmse_ = rmse_/est_num_;
  // calculate the root
  rmse_ = rmse_.array().sqrt();

  return rmse_;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3, 4);

  //recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);
  float range = sqrt(px*px + py*py);
	float range2 = range* range;
	float range3 = range2*range;

  // check not to divided by zero
  if(range<0.0001){
      cout << "CalculateJacobian() - Error - Division by Zero" << endl;

  }
  else{
      Hj <<  px/range,   py/range, 0, 0,
            -py/range2, px/range2, 0, 0,
            py *(vx*py - vy*px)/range3, px*(vy*px - vx*py)/range3, px/range, py /range;
  }

  return Hj;
}
