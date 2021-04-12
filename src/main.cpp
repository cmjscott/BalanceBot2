

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
#include "Logger.h"


SSC32 ssc;
RobotController robot;
TouchScreen screen;
SensorPack sensor;
Controller controller;
Hexapod_Kinematics ik;
KalmanFilter KFx, KFy, KFPlatformX, KFPlatformY;
Vector2d inpU;
Logger varLogger;

angle_t servo_angles[NB_SERVOS];

long long t;
int servo0Feedback = 0;

IntervalTimer testTimer;
long long oldT = 0;
int deltaT = 0;

// offset for the x angle to make the home position totally flat
// y did not need one
//float xOffset = 1.25;


void kalmanTest();
void initKalman();
void intTimer();


void setup()
{
	Serial.begin(115200);
	Serial1.begin(115200);

	


	inpU[0] = 0;
	inpU[1] = 0;


	//Set up robot parameters and attach servos to SSC32
	ssc.begin(Serial1);
	robot.begin(ssc, 0, 1, 2, 4, 5, 6);
	//robot.setBias(radians(-0.75), radians(1.2));

	if(true){
		ssc[0].config(500, 2500, 0, 180, -60, true);
		ssc[1].config(500, 2500, 0, 180, 70, false);
		ssc[2].config(500, 2500, 0, 180, -20, true);
		ssc[3].config(500, 2500, 0, 180, 70, false);
		ssc[4].config(500, 2500, 0, 180, -70, true);
		ssc[5].config(500, 2500, 0, 180, 100, false);

		delay(1000);
		robot.set_pose({0, 0, 10, radians(0), radians(0), radians(0)});
		delay(1000);
		robot.set_pose({0, 0, 0, radians(0), radians(0), radians(0)});
		delay(1000);
	}

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
	controller.setTimestep(0.005);
	controller.begin();


	// must add in order: float, double, int
	varLogger.config(Serial);
	varLogger.appendFloat(&ssc[0].m_pos_deg,"Servo 0 command (degrees)");
	varLogger.appendFloat(&ssc[0].m_pos_ms,"Servo 0 command (ms)");

	varLogger.appendInt(&servo0Feedback,"Servo 0 feedback");

	varLogger.logHeader();
	



	testTimer.begin(intTimer,5000);
	//testTimer.begin(intTimer,1000000);

}

void loop()
{

	/*
	t = millis();

	screen.update();
	sensor.update();
	controller.update();

	//controller.setXDesired(50 * sin(1.0 * t/1000));
	//controller.setYDesired(50 * cos(1.0 * t/1000));

	inpU = controller.getU();
	*/

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

void intTimer()
{
	screen.forceProcess();
	sensor.forceProcess();
	controller.forceProcess();
	inpU = controller.getU();
	robot.set_pose({0,0,0,inpU[0],inpU[1],0});

	//servo0Feedback = analogRead(0);

	//varLogger.log();
	//robot.set_pose_at_point({0,0,0,inpU[0],inpU[1],0},double(screen.getX()),0);
	//robot.set_pose({0,0,0,radians(10),0,0});
}


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
