#ifndef __SENSOR_PACK_H__
#define __SENSOR_PACK_H__

#include <Arduino.h>

//#include "SSC32.h"
#include "PeriodicProcess.h"
#include "TouchScreen.h"


// adafruit 9DOF libraries
#include "Wire.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_LSM303_U.h"
#include "Adafruit_L3GD20_U.h"
#include "Adafruit_9DOF.h"

// adafruit touch screen libraries
#include "Adafruit_STMPE610.h"

//Vector definitions
#include "ArduinoEigen.h"
#include "kalmanfilter.h"

#define SENSOR_SGFILTER_NP 25

////////////////////////////////////////////////////////////////////////////////////////////////////
using namespace Eigen;

class SensorPack : public PeriodicProcess
{
public:
	void config(TouchScreen& _screen);
	void config(TouchScreen& _screen, KalmanFilter& _KFx, KalmanFilter& _KFy);
	void begin();
	void reset();
	void initSensors();

	int isTouched(){return screen->m_isTouched;}

	// Call with getZx(&Zx)
	void getZ(Vector2d& _Zx, Vector2d& _Zy);
	void getPlatformZ(Vector2d& _Zx, Vector2d& _Zy);

	void setBias(double rbX, double rbY, double gbX, double gbY);



	bool hasMeasurement;

protected:
	virtual void process(float timestep);

	Adafruit_9DOF                	dof;
	Adafruit_LSM303_Accel_Unified accel;
	Adafruit_LSM303_Mag_Unified   mag;
	Adafruit_L3GD20_Unified 			gyro;

	sensors_event_t accel_event;
	sensors_event_t mag_event;
	sensors_event_t gyro_event;
	sensors_vec_t   orientation;


	TouchScreen *screen;
	KalmanFilter *KFx, *KFy;
	Vector2d inpU;
	Vector2d Zx, Zy;
	Vector2d statesX, statesY;
	Vector2d integralX, integralY;
	Vector2d Kx, Ky;

	double rollBiasX = 0;
	double rollBiasY = 0;
	double gyroBiasX = 0;
	double gyroBiasY = 0;

	/* Update this with the correct SLP for accurate altitude measurements */
	const float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

	static const double SGFILTER_COEFF0[15];
	static const double SGFILTER_COEFF1[SENSOR_SGFILTER_NP];

	void smoothIMU();
	double applySGFilter(const double samples[], const double coeffs[]);

	double m_xSamples[SENSOR_SGFILTER_NP];
	double m_ySamples[SENSOR_SGFILTER_NP];
};

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif // __YCONTROLLER_H__
