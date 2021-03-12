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

const adafruit_bno055_offsets_t SensorPack::bno_calibration =
{
	-32, -102, -6, -128, 78, -489, 1, -2, 0, 1000, 700,
};

////////////////////////////////////////////////////////////////////////////////////////////////////

void SensorPack::config(TouchScreen& _screen)
{
	screen = &_screen;

	dof   = Adafruit_9DOF();
	accel = Adafruit_LSM303_Accel_Unified(30301);
	mag   = Adafruit_LSM303_Mag_Unified(30302);
	gyro	= Adafruit_L3GD20_Unified(30303);
	bno = Adafruit_BNO055(55);

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
	bno = Adafruit_BNO055(55);

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
	//bno.setSensorOffsets(bno_calibration);
	//displayCalStatus();
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
	_Zx[0] = radians(bno_event.orientation.z);
	_Zx[1] = 0;

	_Zy[0] = radians(bno_event.orientation.y);
	_Zy[1] = 0;

	/*
	_Zx[0] = applySGFilter(m_xSamples,SGFILTER_COEFF0);
	_Zx[1] = radians((gyro_event.gyro.x - gyroBiasX));

	_Zy[0] = applySGFilter(m_ySamples,SGFILTER_COEFF0);
	_Zy[1] = radians((gyro_event.gyro.y - gyroBiasY));
*/
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
	// old sensor code from old IMU
	/*
	accel.getEvent(&accel_event);
	mag.getEvent(&mag_event);
	gyro.getEvent(&gyro_event);
	dof.accelGetOrientation(&accel_event, &orientation);
*/

	bno.getEvent(&bno_event);



	screen->update();

	hasMeasurement = true;

	//smoothIMU();


}

////////////////////////////////////////////////////////////////////////////////////////////////////

void SensorPack::smoothIMU()
{
	for (int i = 0; i < SENSOR_SGFILTER_NP - 1; i++)
	{
		m_xSamples[i] = m_xSamples[i+1];
		m_ySamples[i] = m_ySamples[i+1];
	}

	m_xSamples[SENSOR_SGFILTER_NP - 1] = (orientation.roll - rollBiasX);
	m_ySamples[SENSOR_SGFILTER_NP - 1] = (-orientation.pitch - rollBiasY);

	//m_xSamples[SENSOR_SGFILTER_NP - 1] = (gyro_event.gyro.x - gyroBiasX);
	//m_ySamples[SENSOR_SGFILTER_NP - 1] = (gyro_event.gyro.y - gyroBiasY);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void SensorPack::initSensors()
{
	/*
	if(!accel.begin())
  {
    //There was a problem detecting the LSM303 ... check your connections
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    //while(1);
  }
  if(!mag.begin())
  {
    //There was a problem detecting the LSM303 ... check your connections
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    //while(1);
  }
	if(!gyro.begin(GYRO_RANGE_250DPS))
  {
    //There was a problem detecting the LSM303 ... check your connections
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    //while(1);
  }
	*/
	if(!bno.begin())
	{
		Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
	} else {
		delay(1000);
		resetBNO_offsets();
		bno.setExtCrystalUse(true);
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

void SensorPack::displayCalStatus(void)
{


/* Get the four calibration values (0..3) */
/* Any sensor data reporting 0 should be ignored, */
/* 3 means 'fully calibrated" */
uint8_t system, gyro, accel, mag;
system = gyro = accel = mag = 0;
bno.getCalibration(&system, &gyro, &accel, &mag);
/* The data should be ignored until the system calibration is > 0 */

if(system == 3 && gyro == 3 && accel == 3 && mag == 3 && false){
	adafruit_bno055_offsets_t calib;
	bno.getSensorOffsets(calib);

	Serial.print(calib.accel_offset_x); Serial.print(", ");
	Serial.print(calib.accel_offset_y); Serial.print(", ");
	Serial.print(calib.accel_offset_z); Serial.print(", ");

	Serial.print(calib.mag_offset_x); Serial.print(", ");
	Serial.print(calib.mag_offset_y); Serial.print(", ");
	Serial.print(calib.mag_offset_z); Serial.print(", ");

	Serial.print(calib.gyro_offset_x); Serial.print(", ");
	Serial.print(calib.gyro_offset_y); Serial.print(", ");
	Serial.print(calib.gyro_offset_z); Serial.print(", ");

	Serial.print(calib.accel_radius); Serial.print(", ");

	Serial.print(calib.mag_radius); Serial.print(", ");
	Serial.println("\n");

	//while(1);

} else {
	Serial.print("\t");
	if (!system)
	{
	Serial.print("! ");
	}
	/* Display the individual values */
	Serial.print("Sys:");
	Serial.print(system, DEC);
	Serial.print(" Gyro:");
	Serial.print(gyro, DEC);
	Serial.print(" Accel:");
	Serial.print(accel, DEC);
	Serial.print(" Mag:");
	Serial.print(mag, DEC);

	Serial.print(F(" | Orientation: "));
	Serial.print((float)bno_event.orientation.x);
	Serial.print(F(" "));
	Serial.print((float)bno_event.orientation.y);
	Serial.print(F(" "));
	Serial.print((float)bno_event.orientation.z);
	Serial.println(F(""));
}

}

////////////////////////////////////////////////////////////////////////////////////////////////////

void SensorPack::resetBNO_offsets()
{

	//HARD CODED OFFSETS. BAD PRACTICE, BUT IT WORKS
	adafruit_bno055_offsets_t calibrationData;

	calibrationData.accel_offset_x = -32;
	calibrationData.accel_offset_y = -102;
	calibrationData.accel_offset_z = -6;

	calibrationData.mag_offset_x = -128;
	calibrationData.mag_offset_y = 78;
	calibrationData.mag_offset_z = -489;

	calibrationData.gyro_offset_x = 1;
	calibrationData.gyro_offset_y = -2;
	calibrationData.gyro_offset_z = 0;

	calibrationData.accel_radius = 1000;
	calibrationData.mag_radius = 700;


	//Serial.println("\nCurrent calibration status:");
	//displayCalStatus();

	//Serial.println("\n\nRestoring Calibration data to the BNO055...");
	bno.setSensorOffsets(calibrationData);

	delay(1000);


	/* Optional: Display current status */
	//displaySensorStatus();

 /* Crystal must be configured AFTER loading calibration data into BNO055. */
	//bno.setExtCrystalUse(true);

}

////////////////////////////////////////////////////////////////////////////////////////////////////

void SensorPack::displaySensorStatus(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}
