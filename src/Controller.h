#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include <Arduino.h>

#include "PeriodicProcess.h"
#include "ArduinoEigen.h"
#include "SensorPack.h"
#include "kalmanfilter.h"
#include "Hexapod_Config_1.h"
#include "mathutils.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

class Controller : public PeriodicProcess
{
public:
	void config(SensorPack& _sensor, KalmanFilter& _KFx, KalmanFilter& _KFy);
	void begin();
	void reset();

	double getUx() const {return inpU[0];}
	double getUy() const {return inpU[1];}
	Vector2d getU() const {return inpU;}

	void debug();

	void setOutputLimits(float minOutput, float maxOutput){m_minOutput = minOutput; m_maxOutput = maxOutput;}

	void PIDTest();

	void setXDesired(float xd) {xDesired = xd;}
	void setYDesired(float yd) {yDesired = yd;}

	float getXDesired() const {return xDesired;}
	float getYDesired() const {return yDesired;}

	platform_t calculatedPose;

protected:
	virtual void process(float timestep);

	double ux, uy, xDesired, yDesired;
	float m_minOutput, m_maxOutput;

	//Variables for PID test
	//vector[0] = x values, vector[1] = y values
	Vector2d e2, e1, e0, u2, u1, u0;
	Vector2d a0, a1, a2, b0, b1, b2;
	double kpx, kpy, kix, kiy, kdx, kdy;
	double N, Ts;

	Vector2d inpU;
	Vector2d Zx, Zy, platformZx, platformZy;
	Vector2d statesX, statesY, targetsX, targetsY;
	Vector2d integralX, integralY;
	Vector2d Kx, Ky;
	SensorPack *sensorPack;
	KalmanFilter *KFx, *KFy;

};

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif // __YCONTROLLER_H__
