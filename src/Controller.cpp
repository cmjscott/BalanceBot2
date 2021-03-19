#include "Controller.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
void Controller::config(SensorPack& _sensor, KalmanFilter& _KFx, KalmanFilter& _KFy)
{
  sensorPack = &_sensor;
  KFx = &_KFx;
  KFy = &_KFy;

  xDesired = 0;
  yDesired = 0;
//test
  inpU[0] = 0;
  inpU[1] = 0;

  integralX[0] = 0;
  integralX[1] = 0;

  integralY[0] = 0;
  integralY[1] = 0;

  for (uint8_t i=0; i<2; i++){Kx(i) = KxxVal[i];}
  for (uint8_t i=0; i<2; i++){Ky(i) = KyyVal[i];}

  for (uint8_t i=0; i<2; i++){
    e2(i) = 0;
    e1(i) = 0;
    e0(i) = 0;
    u2(i) = 0;
    u1(i) = 0;
    u0(i) = 0;
  }

  m_minOutput = -radians(11);
  m_maxOutput = radians(11);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::begin()
{
  /*
  kpx = 0.0012;
  kix = 0.0;
  kdx = 0.0005;

  kpy = 0.0012;
  kiy = 0.0;
  kdy = 0.0005;
  */

  kpx = -0.0005;
  kix = -0.0001;
  kdx = -0.0006;

  kpy = -0.0006;
  kiy = -0.0002;
  kdy = -0.0005;

  N = 20;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::reset()
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::process(float timestep)
{
  int t = millis();
  sensorPack->getZ(Zx, Zy);
  sensorPack->getPlatformZ(platformZx, platformZy);

  KFx->predict(inpU);
  KFy->predict(inpU);

  KFx->update(Zx);
  KFy->update(Zy);

  statesX = KFx->getState();
  statesY = KFy->getState();

  targetsX = statesX;
  targetsY = statesY;

  //targetsX = Zx;
  //targetsY = Zy;

  targetsX[0] = targetsX[0] - xDesired;
  targetsY[0] = targetsY[0] - yDesired;

  double intTest = 0.003;
  double satTarget = 0.1 / intTest;
  intTest = 0;

  integralX[0] = saturate(integralX[0] + (targetsX[0] * getTimestep()),-satTarget,satTarget) * sensorPack->isTouched();
  integralY[0] = saturate(integralY[0] + (targetsY[0] * getTimestep()),-satTarget,satTarget) * sensorPack->isTouched();



  //inpU[0] = 10 * sin(3.0 * t/1000);
  //inpU[0] = 10;
  //inpU[1] = 10;




  //inpU[0] = saturate(-Kx.transpose() * targetsX - intTest*integralX[0], m_minOutput, m_maxOutput);
  //inpU[1] = saturate(-Ky.transpose() * targetsY - intTest*integralY[0], m_minOutput, m_maxOutput);

  //inpU[0] = saturate(-Kx.transpose() * targetsX + saturate(intTest*integralX[0],-0.05,0.05), m_minOutput, m_maxOutput);
  //inpU[1] = saturate(-Ky.transpose() * targetsY + saturate(intTest*integralY[0],-0.05,0.05), m_minOutput, m_maxOutput);

  PIDTest();
  inpU[0] = saturate(u0[0], m_minOutput, m_maxOutput);
  inpU[1] = saturate(u0[1], m_minOutput, m_maxOutput);
  debug();
  //sensorPack->displayCalStatus();
	//ux = ctrlx->compute(xDesired, screen->getX(), timestep);
	//uy = ctrly->compute(yDesired, screen->getY(), timestep);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::PIDTest()
{
  Ts = getTimestep();

  //X values
  a0[0] = (1+N*Ts);
  a1[0] = -(2 + N*Ts);
  a2[0] = 1;
  b0[0] = kpx*(1+N*Ts) + kix*Ts*(1+N*Ts) + kdx*N;
  b1[0] = -(kpx*(2+N*Ts) + kix*Ts + 2*kdx*N);
  b2[0] = kpx + kdx*N;

  //Y values
  a0[1] = (1+N*Ts);
  a1[1] = -(2 + N*Ts);
  a2[1] = 1;
  b0[1] = kpy*(1+N*Ts) + kiy*Ts*(1+N*Ts) + kdy*N;
  b1[1] = -(kpy*(2+N*Ts) + kiy*Ts + 2*kdy*N);
  b2[1] = kpy + kdy*N;


  // update variables
  e2=e1;
  e1=e0;
  u2=u1;
  u1=u0;

  //  update outputs for X
  e0[0] = targetsX[0];
  u0[0] = -(a1[0] / a0[0]) * u1[0] - (a2[0] / a0[0]) * u2[0] + (b0[0]/a0[0])*e0[0] + (b1[0]/a0[0])*e1[0] + (b2[0]/a0[0])*e2[0];

  // update outputs for Y
  e0[1] = targetsY[0];
  u0[1] = -(a1[1]/a0[1])*u1[1] -(a2[1]/a0[1])*u2[1] + (b0[1]/a0[1])*e0[1] + (b1[1]/a0[1])*e1[1] + (b2[1]/a0[1])*e2[1];

}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::debug()
{
  long long t = millis();
  //Serial.print(double(t/1000.0),4); Serial.print(", ");

/*
  Serial.print(statesX[0]); Serial.print(", ");
  Serial.print(Zx[0]); Serial.print(", ");
  Serial.print(statesX[1]); Serial.print(", ");
  Serial.print(Zx[1]); Serial.print(", ");
  Serial.print(degrees(inpU[0])); Serial.print(", ");
  Serial.print(0);
*/

  //Serial.print(degrees(platformZx[0])); Serial.print(", ");
  //Serial.print(degrees(platformZy[0])); Serial.print(", ");


  //Serial.print(statesY[0]); Serial.print(", ");
  //Serial.print(Zy[0]); Serial.print(", ");
  //Serial.print(statesY[1]); Serial.print(", ");
  //Serial.print(Zy[1]); Serial.print(", ");
  //Serial.print(inpU[1]);


/*
Serial.print(platformZx[0]); Serial.print(", ");
Serial.print(platformZx[1]); Serial.print(", ");
Serial.print(platformZy[0]); Serial.print(", ");
Serial.print(platformZy[1]); Serial.print(", ");
Serial.print(degrees(inpU[0])); Serial.print(", ");
Serial.print(degrees(inpU[1])); Serial.print(", ");
*/


Serial.print(targetsX[0]); Serial.print(", ");
Serial.print(statesX[0]); Serial.print(", ");
Serial.print(degrees(inpU[0])); Serial.print(", ");
Serial.print(targetsY[0]); Serial.print(", ");
Serial.print(statesY[0]); Serial.print(", ");
Serial.print(degrees(inpU[1])); Serial.print(", ");
//Serial.print(getTimestep()); Serial.print(", ");


/*
Serial.print(e0[0]); Serial.print(", ");
//Serial.print(e1[0]); Serial.print(", ");
//Serial.print(e2[0]); Serial.print(", ");
Serial.print(u0[0]); Serial.print(", ");
Serial.print(u1[0]); Serial.print(", ");
Serial.print(u2[0]); Serial.print(", ");
//Serial.print(getTimestep()); Serial.print(", ");
*/

//Serial.print(inpU[0]); Serial.print(", ");
//Serial.print(inpU[1]); Serial.print(", ");
//Serial.print(-10); Serial.print(", ");
//Serial.print(10); Serial.print(", ");

//Serial.print(statesX[0]); Serial.print(", ");
//Serial.print(Zx[0]); Serial.print(", ");
//Serial.print(statesX[1]); Serial.print(", ");
//Serial.print(Zx[1]); Serial.print(", ");

/*
Serial.print(statesX[0]); Serial.print(", ");
Serial.print(Zx[0]); Serial.print(", ");
Serial.print(statesX[1]); Serial.print(", ");
Serial.print(Zx[1]); Serial.print(", ");
Serial.print(integralX[0]); Serial.print(", ");
Serial.print(xDesired);Serial.print(", ");
Serial.print(inpU[0]);
*/

/*
Serial.print(Zx[0]); Serial.print(", ");
Serial.print(degrees(-Kx[0] * Zx[0])); Serial.print(", ");
Serial.print(Zx[1] / 100.0); Serial.print(", ");
Serial.print(degrees(-Kx[1] * Zx[1])); Serial.print(", ");
Serial.print(integralX[0]); Serial.print(", ");
Serial.print(degrees(intTest * integralX[0])); Serial.print(", ");
Serial.print(inpU[0]);
*/

/*
Serial.print(statesY[0]/1000); Serial.print(", ");
Serial.print(Zy[0]/1000); Serial.print(", ");
Serial.print(statesY[1]/1000); Serial.print(", ");
Serial.print(Zy[1]/1000);
*/

  Serial.print("\n");
}

////////////////////////////////////////////////////////////////////////////////////////////////////
