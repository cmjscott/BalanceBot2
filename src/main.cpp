

#include <Arduino.h>
#include <math.h>

#include "SSC32.h"
#include "RobotController.h"
#include "TouchScreen.h"
#include "Controller.h"
#include "PID.h"
#include "Hexapod_Kinematics.h"
#include "Hexapod_Config_1.h"
#include "SensorPack.h"
#include "kalmanfilter.h"

// adafruit 9DOF libraries
//#include "Wire.h"
//#include "Adafruit_Sensor.h"
//#include "Adafruit_LSM303_U.h"
//#include "Adafruit_L3GD20_U.h"
//#include "Adafruit_9DOF.h"
//#include "Adafruit_BNO055.h"

// adafruit touch screen libraries
//#include "Adafruit_STMPE610.h"



//#include "ArduinoEigen.h"



/*
// Assign a unique ID to the sensors
Adafruit_9DOF                	dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified				gyro 	= Adafruit_L3GD20_Unified(30303);

Adafruit_STMPE610 touch = Adafruit_STMPE610();

sensors_event_t accel_event;
sensors_event_t mag_event;
sensors_event_t gyro_event;
sensors_vec_t   orientation;

// Update this with the correct SLP for accurate altitude measurements
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

*/
SSC32 ssc;
RobotController robot;
TouchScreen screen;
SensorPack sensor;
Controller controller;
Hexapod_Kinematics ik;
KalmanFilter KFx, KFy, KFPlatformX, KFPlatformY;
Vector2d inpU;


angle_t servo_angles[NB_SERVOS];

long long t;

int val;
bool val2 = true;

// offset for the x angle to make the home position totally flat
// y did not need one
//float xOffset = 1.25;


void kalmanTest();
void initKalman();
//void step_input();
//void demo();
//void test();
//void rotationTest();
//void rotationTest2();
//void printIMU();
//void initSensors();


void setup()
{
	Serial.begin(115200);
	Serial.println("x, xDot, y, yDot, ux, uy");
	Serial1.begin(115200);



	inpU[0] = 0;
	inpU[1] = 0;


	//Set up robot parameters and attach servos to SSC32
	ssc.begin(Serial1);
	robot.begin(ssc, 0, 1, 2, 4, 5, 6);
	robot.setBias(radians(-0.75), radians(1.2));

	//Setup for touch screen Test
	//touch.begin(0x41);
	//ssc[5].config(500, 2500, 0, 180, 0, true);
	//ssc[5].set_degrees(120);
	//ssc.commit();

	//screen.config xMax, xMin, xLength, yMax, yMin, yLength);
	screen.config(3875, 200, 228, 4000, 75, 304); // milimeters
	screen.enable();
	screen.setTimestep(0.005);
	screen.begin();

	initKalman();

	sensor.config(screen, KFPlatformX, KFPlatformY);
	sensor.setBias(-0.45, 0.3, -0.01, -0.08);
	sensor.enable();
	sensor.setTimestep(0.005);
	sensor.begin();


	controller.config(sensor, KFx, KFy);
	controller.enable();
	controller.setTimestep(0.015);
	controller.begin();

	if(true){
		ssc[0].config(500, 2500, 0, 180, -60, true);
		ssc[1].config(500, 2500, 0, 180, 70, false);
		ssc[2].config(500, 2500, 0, 180, -20, true);
		ssc[3].config(500, 2500, 0, 180, 70, false);
		ssc[4].config(500, 2500, 0, 180, -70, true);
		ssc[5].config(500, 2500, 0, 180, 100, false);

		//Ensure that everything is set up before homing robot
		delay(1000);
		robot.set_pose({0, 0, 10, radians(0), radians(0), radians(0)});
		delay(1000);
		//robot.set_pose({0, 0, 0, radians(0.75), radians(-1.2), radians(0)});
		robot.set_pose({0, 0, 0, radians(0), radians(0), radians(0)});
		delay(1000);
		//controller.setXDesired(30.0);
		//controller.setYDesired(30.0);
	}



	//initSensors();
	//Serial.println("t, xMeasure, rollMeasure, gyroMeasure, x, xdot, roll, gyro");
	//Serial.println("x, zx, xdot, zxdot, theta, zTheta, thetaDot, zThetaDot, input");
	//Serial.println("t, x, zx, xdot, zxdot, ux, y, zy, ydot, zydot, uy");

	Serial.println("x, ux, y, uy");
	//Serial.println("x, Kx, xDot, K_xDot, integral, K_integral, U");
	//Serial.println("zX, zY");
	//sensor.displayCalStatus();
	//Serial.println("t, y, Zy, yDot, ZyDot");

	//Serial.println("t, y, zy, ydot, zydot");

	//Serial.println("thetaX, thetaDotX, thetaY, thetaDotY, ux, uy");
	//Serial.println(F("Adafruit 9 DOF Pitch/Roll/Heading Example")); Serial.println("");
	//Serial.println("t, inp x, inp y, out x,out y, rollx, rolly");
	//Serial.println("inpx, inpy, rollx,rolly, gyrox, gyroy");
	//Serial.println("t, inpx, inpy, rollx,rolly, gyrox, gyroy");
	//Serial.println("inpx, rollx, roll_Kalman, gyrox, gyro_Kalman");
	//Serial.println("x, y, z");
	//Serial.println("t, x, y, z");
	//Serial.println("x, xdot, theta, thetadot");
	//Serial.print(int(millis())); Serial.print(",");
	//printIMU();

	/* Initialise the sensors */
	//servo home positions: 1540, 1500, 1440, 1550, 1440, 1550
}

void loop()
{
	t = millis();

	screen.update();
	sensor.update();
	controller.update();
	//controller.setXDesired(50 * sin(1.0 * t/1000));
	//controller.setYDesired(50 * cos(1.0 * t/1000));

	inpU = controller.getU();
	//robot.set_pose({0,0,0,inpU[0],inpU[1],0});

	//robot.set_pose({20 * sin(3.0 * t/1000), 20 * cos(8.0 * t/1000), 0, 0, 0, 0});
	//robot.set_pose({0, 0, 0, radians(10 * sin(5.0 * t/1000)), radians(10 * sin(3.0 * t/1000)), 0});
	//robot.set_pose({0, 0, 0, radians(10 * sin(1.0 * t/1000)), radians(10 * sin(0.7 * t/1000)), 0});
	//robot.set_pose({0, 30 * sin(5.0 * t/1000), 6 * sin(3.0 * t/1000), 0, 0, 0});
	//robot.set_pose({-0.4106,1.5192,-2.3289,radians(5),radians(5),0});
	//delay(2000);
	//robot.set_pose({0,0,15,radians(10),radians(10),0});
	//delay(2000);

	//test();
	//sine_input();
	//delay(10);
	//Serial.print(int(t)); Serial.print(",");
	//printIMU();
	//while(1);
	//printIMU();

  //Serial.print(val);

	// Set this to true to feed a sine input to the Controller
	// Makes the ball roll in circles
	// Could also use the step input function
	//if ((t > 15 * 1000) && false){sine_input();}

	// THIS IS WHERE THE ACTUAL CONTROL HAPPENS
	//robot.set_pose(0,30,-controller.getUx() + xOffset, -controller.getUy());
//step_input();



}



// NOT USED
/*
void step_input()
{
	if (t > 4 * 1000 && val2)
	{
		ssc[0].set_degrees(180);
		ssc.commit(1000);
		val2 = false;
	}
	val = analogRead(9);
	Serial.print(int(t)); Serial.print(", ");
	Serial.print(ssc[0].get_degrees()); Serial.print(", ");
	Serial.print(ssc[0].get_position()); Serial.print(", ");
	Serial.print(val);
	Serial.print("\n");

	delay(10);
}
*/

// NOT USED
/*
void test()
{
	uint16_t x, y;
  uint8_t z;
	t = millis();
  if (touch.touched()) {
    // read x & y & z;
    while (! touch.bufferEmpty()) {
      //Serial.print(touch.bufferSize());
			Serial.print(int(t)); Serial.print(",");
      touch.readData(&x, &y, &z);
      Serial.print(x); Serial.print(", ");
      Serial.print(y); Serial.print(", ");
      Serial.print(z);
			Serial.print("\n");
    }
    touch.writeRegister8(STMPE_INT_STA, 0xFF); // reset all ints, in this example unneeded depending in use
  }
}
*/

// NOT USED
/*
void demo()
{
	const platform_t coords[] = {
			// X
			{HX_X_MAX, 0, 0, 0, 0, 0},
			{HX_X_MIN, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0},

			// Y
			{0, HX_Y_MAX, 0, 0, 0, 0},
			{0, HX_Y_MIN, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0},

			// Z
			{0, 0, HX_Z_MAX, 0, 0, 0},
			{0, 0, HX_Z_MIN, 0, 0, 0},
			{0, 0, 0, 0, 0, 0},

			// A
			{0, 0, 0, HX_A_MAX, 0, 0},
			{0, 0, 0, HX_A_MIN, 0, 0},
			{0, 0, 0, 0, 0, 0},

			// B
			{0, 0, 0, 0, HX_B_MAX, 0},
			{0, 0, 0, 0, HX_B_MIN, 0},
			{0, 0, 0, 0, 0, 0},

			// C
			{0, 0, 0, 0, 0, HX_C_MAX},
			{0, 0, 0, 0, 0, HX_C_MIN},
			{0, 0, 0, 0, 0, 0}};

	int8_t movOK = -1;
	for (uint8_t cnt = 0; cnt < COUNT_OF(coords); cnt++)
	{
		movOK = ik.calcServoAnglesOld(coords[cnt], servo_angles);

		for (int i=0; i < NB_SERVOS; i++) {
			ssc[i].set_degrees(servo_angles[i].deg);
			Serial.print("Servo ");
			Serial.print(i);
			Serial.print(": ");
			Serial.print(servo_angles[i].deg);
			Serial.print(" | ");
		}
		ssc.commit();
		Serial.println("");
		delay(2000);
			}
}
*/

// NOT USED
/*
void rotationTest()
{
	double angles[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

	int8_t movOK = -1;
	double xDist, zDist;

	for (uint8_t cnt = 0; cnt < COUNT_OF(angles); cnt++)
	{
		xDist = -(60 - 60 * cos(radians(angles[cnt])));
		zDist = -(60 * sin(radians(angles[cnt])));

		movOK = ik.calcServoAnglesOld({xDist, 0, zDist, 0, radians(angles[cnt]), 0} , servo_angles);

		for (int i=0; i < NB_SERVOS; i++) {
			ssc[i].set_degrees(servo_angles[i].deg);
			Serial.print("Servo ");
			Serial.print(i);
			Serial.print(": ");
			Serial.print(servo_angles[i].deg);
			Serial.print(" | ");
		}
		ssc.commit();
		Serial.println("");
		delay(50);
	}

	for (uint8_t cnt = COUNT_OF(angles) - 1; cnt > 0; cnt--)
	{
		xDist = -(60 - 60 * cos(radians(angles[cnt])));
		zDist = -(60 * sin(radians(angles[cnt])));

		movOK = ik.calcServoAnglesOld({xDist, 0, zDist, 0, radians(angles[cnt]), 0} , servo_angles);

		for (int i=0; i < NB_SERVOS; i++) {
			ssc[i].set_degrees(servo_angles[i].deg);
			Serial.print("Servo ");
			Serial.print(i);
			Serial.print(": ");
			Serial.print(servo_angles[i].deg);
			Serial.print(" | ");
		}
		ssc.commit();
		Serial.println("");
		delay(50);
	}

}
*/

// NOT USED
/*
void rotationTest2()
{
	double angles[] = {-9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
	//double angles[] = {1,2};
	//double angles[] = {-1, 1};

	for (uint8_t cnt = 0; cnt < COUNT_OF(angles); cnt++)
	{
		robot.set_pose({0, 0, 0, radians(angles[cnt]), 0, 0});
		delay(30);
		printIMU();
	}
	//delay(1000);
	for (uint8_t cnt = COUNT_OF(angles) - 2; cnt > 0; cnt--)
	{
		robot.set_pose({0, 0, 0, radians(angles[cnt]), 0, 0});
		delay(30);
		printIMU();
	}
	//delay(1000);
}
*/

// NOT USED
/*
void printIMU()
{
	// Calculate pitch and roll from the raw accelerometer data
	accel.getEvent(&accel_event);
	mag.getEvent(&mag_event);
	gyro.getEvent(&gyro_event);

	Serial.print(degrees(robot.m_ik.getHX_A()));
	Serial.print(",");
	Serial.print(degrees(robot.m_ik.getHX_B()));
	Serial.print(",");

	if (dof.accelGetOrientation(&accel_event, &orientation)&& true)
	{
		// 'orientation' should have valid .roll and .pitch fields
		Serial.print(-orientation.roll);
		Serial.print(",");
		Serial.print(orientation.pitch);
		Serial.print(",");
	}

	Serial.print(degrees(-gyro_event.gyro.x));
	Serial.print(",");
	Serial.print(degrees(-gyro_event.gyro.y));


	Serial.print("\n");
}
*/

// NOT USED
/*
void initSensors()
{
  if(!accel.begin())
  {
    // There was a problem detecting the LSM303 ... check your connections
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    // There was a problem detecting the LSM303 ... check your connections
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
	if(!gyro.begin(GYRO_RANGE_2000DPS))
  {
    // There was a problem detecting the LSM303 ... check your connections
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }

	if (!touch.begin(0x41)) {
		Serial.println("STMPE not found!");
		//while(1);
	}
	Serial.println("Waiting for touch sense");
}
*/

void initKalman()
{

	if(!KFx.start(2, XVal, PxxVal, AxxVal, BxxVal, HxxVal, QxxVal, RxxVal))
	{
		Serial.println("Something is fucked with the kalman filter X");
	} else {
		//Serial.println("Kalman initialization is all good");
	}

	if(!KFy.start(2, XVal, PyyVal, AyyVal, ByyVal, HyyVal, QyyVal, RyyVal))
	{
		Serial.println("Something is fucked with the kalman filter Y");
	} else {
		//Serial.println("Kalman initialization is all good");
	}

	if(!KFPlatformX.start(2, XVal, PPxxVal, APxxVal, BPxxVal, HPxxVal, QPxxVal, RPxxVal))
	{
		Serial.println("Something is fucked with the kalman filter PX");
	} else {
		//Serial.println("Kalman initialization is all good");
	}

	if(!KFPlatformY.start(2, XVal, PPxxVal, APxxVal, BPxxVal, HPxxVal, QPxxVal, RPxxVal))
	{
		Serial.println("Something is fucked with the kalman filter PY");
	} else {
		//Serial.println("Kalman initialization is all good");
	}
}
