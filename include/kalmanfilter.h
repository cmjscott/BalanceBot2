#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_

#include "ArduinoEigen.h"
//#include "datapoint.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter{

  private:
    int _n;
    VectorXd _x;
    MatrixXd _Pmat;
    MatrixXd _A;
    MatrixXd _Bmat;
    MatrixXd _H;
    MatrixXd _Q;
    MatrixXd _R;
    MatrixXd _I;

  public:
    KalmanFilter(){};
    bool start(const int nin, const VectorXd xin, const MatrixXd Pin, const MatrixXd Ain, const MatrixXd Bin, const MatrixXd Hin, const MatrixXd Qin, const MatrixXd Rin);
    bool start(const int nin, const double xin[], const double Pin[], const double Ain[], const double Bin[], const double Hin[], const double Qin[], const double Rin[]);
    void setQ(const MatrixXd& Qin);
    void updateA(const double dt);
    VectorXd getState() const;
    void predictEKF();
    void predict(const VectorXd& u);
    //void predict(const )
    void updateEKF(const VectorXd& z, const MatrixXd& H, const VectorXd& Hx, const MatrixXd& R);
    void update(const VectorXd& z);
};


#endif /* KALMANFILTER_H_ */
