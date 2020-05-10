#include "kalmanfilter.h"
#include "Hexapod_Config_1.h"

bool KalmanFilter::start(
  const int nin,
  const VectorXd xin,
  const MatrixXd Pin,
  const MatrixXd Ain,
  const MatrixXd Bin,
  const MatrixXd Hin,
  const MatrixXd Qin,
  const MatrixXd Rin){

  _n = nin;
  _I = MatrixXd::Identity(_n, _n);
  _x = xin;
  _Pmat = Pin;

  _A = Ain;
  _Bmat = Bin;
  _H = Hin;
  _Q = Qin;
  _R = Rin;

  return true;
}

bool KalmanFilter::start(const int nin, const double xin[], const double Pin[], const double Ain[], const double Bin[], const double Hin[], const double Qin[], const double Rin[])
{
  _n = nin;
  _I = MatrixXd::Identity(_n, _n);

  _x = VectorXd(2);
  _Pmat = MatrixXd(2,2);
  _A = MatrixXd(2,2);
  _Bmat = MatrixXd(2,2);
  _H = MatrixXd(2,2);
  _Q = MatrixXd(2,2);
  _R = MatrixXd(2,2);

  for (uint8_t i=0; i<4; i++){_Pmat(i) = Pin[i];}
  for (uint8_t i=0; i<4; i++){_A(i) = Ain[i];}
  for (uint8_t i=0; i<4; i++){_Bmat(i) = Bin[i];}
  for (uint8_t i=0; i<4; i++){_H(i) = Hin[i];}
  for (uint8_t i=0; i<4; i++){_Q(i) = Qin[i];}
  for (uint8_t i=0; i<4; i++){_R(i) = Rin[i];}
  for (uint8_t i=0; i<2; i++){_x(i) = xin[i];}

  _Pmat.transposeInPlace();
  _A.transposeInPlace();
  _H.transposeInPlace();
  _Q.transposeInPlace();
  _R.transposeInPlace();
/*
  Serial.print(_A(0,0)); Serial.print(", ");
  Serial.print(_A(1,0)); Serial.print(", ");

  Serial.print(_A(2,0)); Serial.print(", ");
  Serial.print(_A(3,0)); Serial.println("");

  Serial.print(_Bmat(0,0)); Serial.print(", ");
  Serial.print(_Bmat(0,1)); Serial.println("");

  Serial.print(_Bmat(1,0)); Serial.print(", ");
  Serial.print(_Bmat(1,1)); Serial.println("");

  Serial.print(_Bmat(2,0)); Serial.print(", ");
  Serial.print(_Bmat(2,1)); Serial.println("");

  Serial.print(_Bmat(3,0)); Serial.print(", ");
  Serial.print(_Bmat(3,1)); Serial.println("");
*/
  return true;
}

void KalmanFilter::setQ(const MatrixXd& Qin){
  _Q = Qin;
}

void KalmanFilter::updateA(const double dt){
  _A(0, 2) = dt;
  _A(1, 3) = dt;
}

VectorXd KalmanFilter::getState() const{
  return _x;
}


void KalmanFilter::predict(const VectorXd& u){
  _x = _A * _x + _Bmat * u;
  _Pmat = _A * _Pmat * _A.transpose() + _Q;
}


void KalmanFilter::update(const VectorXd& z){

  const MatrixXd PHt = _Pmat * _H.transpose();
  const MatrixXd S = _H * PHt + _R;
  const MatrixXd K = PHt * S.inverse();
  const MatrixXd Hx = _H * _x;

  VectorXd y = z - Hx;

  _x = _x + K * y;
  _Pmat = (_I - K * _H) * _Pmat;
}
