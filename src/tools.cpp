#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

//The code for this function was taken from the instruction videos.
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()
     || estimations.size() == 0){
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  //accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){

    VectorXd residual = estimations[i] - ground_truth[i];

    //coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  //calculate the mean
  rmse = rmse/estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

VectorXd Tools::CTRVModelUpdate(double deltaT, VectorXd sigmaPoint){
  VectorXd toReturn = VectorXd(5);

  double px = sigmaPoint(0);
  double py = sigmaPoint(1);
  double v = sigmaPoint(2);
  double yaw = sigmaPoint(3);
  double yaw_rate = sigmaPoint(4);
  double noise_a = sigmaPoint(5);
  double noise_yaw = sigmaPoint(6);

  double px2, py2, v2, yaw2, yaw_rate2;

  //Update without noise
  if(fabs(yaw_rate) < 0.0001){
    px2 = px + cos(yaw) * v * deltaT;
    py2 = py + sin(yaw) * v * deltaT;
  }else{
    px2 = px + v/yaw_rate * (sin(yaw + yaw_rate * deltaT) - sin(yaw));
    py2 = py + v/yaw_rate * (-cos(yaw + yaw_rate * deltaT) + cos(yaw));
  }

  v2 = v;
  yaw2 = yaw + yaw_rate * deltaT;
  yaw_rate2 = yaw_rate;

  //Add noise
  px2 += 1/2 * pow(deltaT, 2) * noise_a * cos(yaw);
  py2 += 1/2 * pow(deltaT, 2) * noise_a * sin(yaw);
  v2 += deltaT * noise_a;
  yaw2 += 1/2 * pow(deltaT, 2) * noise_yaw;
  yaw_rate2 += deltaT * noise_yaw;

  toReturn(0) = px2;
  toReturn(1) = py2;
  toReturn(2) = v2;
  toReturn(3) = yaw2;
  toReturn(4) = yaw_rate2;

  return toReturn;
}
