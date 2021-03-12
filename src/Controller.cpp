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

  m_minOutput = -radians(10);
  m_maxOutput = radians(10);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::begin()
{

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


  inpU[0] = saturate(-Kx.transpose() * targetsX - intTest*integralX[0], m_minOutput, m_maxOutput);
  inpU[1] = saturate(-Ky.transpose() * targetsY - intTest*integralY[0], m_minOutput, m_maxOutput);

  //inpU[0] = saturate(-Kx.transpose() * targetsX + saturate(intTest*integralX[0],-0.05,0.05), m_minOutput, m_maxOutput);
  //inpU[1] = saturate(-Ky.transpose() * targetsY + saturate(intTest*integralY[0],-0.05,0.05), m_minOutput, m_maxOutput);

  debug();
  //sensorPack->displayCalStatus();
	//ux = ctrlx->compute(xDesired, screen->getX(), timestep);
	//uy = ctrly->compute(yDesired, screen->getY(), timestep);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::calculatePose()
{

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



Serial.print(platformZx[0]); Serial.print(", ");
Serial.print(platformZx[1]); Serial.print(", ");
Serial.print(platformZy[0]); Serial.print(", ");
Serial.print(platformZy[1]); Serial.print(", ");
Serial.print(degrees(inpU[0])); Serial.print(", ");
Serial.print(degrees(inpU[1])); Serial.print(", ");


/*
Serial.print(targetsX[0]); Serial.print(", ");
Serial.print(targetsX[1]); Serial.print(", ");
Serial.print(targetsY[0]); Serial.print(", ");
Serial.print(targetsY[1]); Serial.print(", ");
Serial.print(degrees(inpU[0])); Serial.print(", ");
Serial.print(degrees(inpU[1])); Serial.print(", ");
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
