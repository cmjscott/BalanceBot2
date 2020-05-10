#include "SensorPack.h"

const double SensorPack::SGFILTER_COEFF0[] =
{
	-78.0 / 1105,
	-13.0 / 1105,
   42.0 / 1105,
	 87.0 / 1105,
	122.0 / 1105,
	147.0 / 1105,
	162.0 / 1105,
	167.0 / 1105,
	162.0 / 1105,
	147.0 / 1105,
	122.0 / 1105,
	 87.0 / 1105,
   42.0 / 1105,
	-13.0 / 1105,
	-78.0 / 1105,
};

const double SensorPack::SGFILTER_COEFF1[] =
{
	-253.0 / 5175,
	-138.0 / 5175,
	 -33.0 / 5175,
	  62.0 / 5175,
	 147.0 / 5175,
	 222.0 / 5175,
	 287.0 / 5175,
	 343.0 / 5175,
	 387.0 / 5175,
	 422.0 / 5175,
	 447.0 / 5175,
	 462.0 / 5175,
	 467.0 / 5175,
	 462.0 / 5175,
	 447.0 / 5175,
	 422.0 / 5175,
	 387.0 / 5175,
	 343.0 / 5175,
	 287.0 / 5175,
	 222.0 / 5175,
	 147.0 / 5175,
	  62.0 / 5175,
	 -33.0 / 5175,
	-138.0 / 5175,
	-253.0 / 5175,
};

////////////////////////////////////////////////////////////////////////////////////////////////////

void SensorPack::config(TouchScreen& _screen)
{
	screen = &_screen;

	dof   = Adafruit_9DOF();
	accel = Adafruit_LSM303_Accel_Unified(30301);
	mag   = Adafruit_LSM303_Mag_Unified(30302);
	gyro	= Adafruit_L3GD20_Unified(30303);

	/* Update this with the correct SLP for accurate altitude measurements */

	hasMeasurement = false;
	initSensors();

	rollBiasX = 0;
	rollBiasY = 0;
	gyroBiasX = 0;
	gyroBiasY = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void SensorPack::config(TouchScreen& _screen, KalmanFilter& _KFx, KalmanFilter& _KFy)
{
	screen = &_screen;
	KFx = &_KFx;
	KFy = &_KFy;

	dof   = Adafruit_9DOF();
	accel = Adafruit_LSM303_Accel_Unified(30301);
	mag   = Adafruit_LSM303_Mag_Unified(30302);
	gyro	= Adafruit_L3GD20_Unified(30303);

	/* Update this with the correct SLP for accurate altitude measurements */

	hasMeasurement = false;
	initSensors();

	rollBiasX = 0;
	rollBiasY = 0;
	gyroBiasX = 0;
	gyroBiasY = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void SensorPack::begin()
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////

void SensorPack::reset()
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////

void SensorPack::setBias(double rbX, double rbY, double gbX, double gbY)
{
	rollBiasX = rbX;
	rollBiasY = rbY;
	gyroBiasX = gbX;
	gyroBiasY = gbY;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void SensorPack::getZ(Vector2d& _Zx, Vector2d& _Zy)
{
	//if(isUpdated())
	if(true)
	{
		// <position, angle, gyro>
		_Zx[0] = screen->getX();
		_Zx[1] = screen->getDX();
		//_Zx[2] = (-orientation.roll - rollBiasX);
		//_Zx[3] = (degrees(-gyro_event.gyro.x) - gyroBiasX);

		_Zy[0] = screen->getY();
		_Zy[1] = screen->getDY();
		//_Zy[2] = (orientation.pitch - rollBiasY);
		//_Zy[3] = (degrees(-gyro_event.gyro.y) - gyroBiasY);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void SensorPack::getPlatformZ(Vector2d& _Zx, Vector2d& _Zy)
{

	_Zx[0] = applySGFilter(m_xSamples,SGFILTER_COEFF1);
	_Zx[1] = (gyro_event.gyro.x - gyroBiasX);

	_Zy[0] = applySGFilter(m_ySamples,SGFILTER_COEFF1);
	_Zy[1] = (gyro_event.gyro.y - gyroBiasY);

/*
		// <position, angle, gyro>
		_Zx[0] = (orientation.roll - rollBiasX);
		_Zx[1] = (gyro_event.gyro.x - gyroBiasX);

		_Zy[0] = (-orientation.pitch - rollBiasY);
		_Zy[1] = (gyro_event.gyro.y - gyroBiasY);
*/
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void SensorPack::process(float timestep)
{
	/* Calculate pitch and roll from the raw accelerometer data */
	accel.getEvent(&accel_event);
	mag.getEvent(&mag_event);
	gyro.getEvent(&gyro_event);

	dof.accelGetOrientation(&accel_event, &orientation);

	screen->update();

	hasMeasurement = true;

	smoothIMU();


}

////////////////////////////////////////////////////////////////////////////////////////////////////

void SensorPack::smoothIMU()
{
	for (int i = 0; i < SENSOR_SGFILTER_NP - 1; i++)
	{
		m_xSamples[i] = m_xSamples[i+1];
		m_ySamples[i] = m_ySamples[i+1];
	}

	//m_xSamples[SENSOR_SGFILTER_NP - 1] = (orientation.roll - rollBiasX);
	//m_ySamples[SENSOR_SGFILTER_NP - 1] = (-orientation.pitch - rollBiasY);

	m_xSamples[SENSOR_SGFILTER_NP - 1] = (gyro_event.gyro.x - gyroBiasX);
	m_ySamples[SENSOR_SGFILTER_NP - 1] = (gyro_event.gyro.y - gyroBiasY);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void SensorPack::initSensors()
{
	if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    //while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    //while(1);
  }
	if(!gyro.begin(GYRO_RANGE_250DPS))
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    //while(1);
  }
	//Serial.println("Sensors sucessfully initialized!");
}

////////////////////////////////////////////////////////////////////////////////////////////////////

double SensorPack::applySGFilter(const double samples[], const double coeffs[])
{
	double output = 0;
	for (int i = 0; i < SENSOR_SGFILTER_NP; i++)
		output += samples[i] * coeffs[i];
	return output;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
