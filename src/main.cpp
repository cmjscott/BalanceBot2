



#include "main.h"



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

int servo_0_Feedback = 0;
int servo_1_Feedback = 0;
int servo_2_Feedback = 0;
int servo_3_Feedback = 0;
int servo_4_Feedback = 0;
int servo_5_Feedback = 0;

int servo_0_pin = A0;
int servo_1_pin = A1;
int servo_2_pin = A2;
int servo_3_pin = A3;
int servo_4_pin = A6;
int servo_5_pin = A7;

double platformHeight = 0;


IntervalTimer testTimer;
long long oldT = 0;
int deltaT = 0;

// offset for the x angle to make the home position totally flat
// y did not need one
//float xOffset = 1.25;


void kalmanTest();
void initKalman();
void interruptTimer();


void setup()
{
	Serial.begin(BPS);
	Serial1.begin(BPS);


	inpU[0] = 0;
	inpU[1] = 0;


	//Set up robot parameters and attach servos to SSC32
	ssc.begin(Serial1);
	robot.begin(ssc, 0, 1, 2, 4, 5, 6);
	//robot.setBias(radians(-0.75), radians(1.2));

	ssc[0].config(servo_min_duty, servo_max_duty, servo_min_degrees, servo_max_degrees, servo_0_offset, servo_CCW_positive);
	ssc[1].config(servo_min_duty, servo_max_duty, servo_min_degrees, servo_max_degrees, servo_1_offset, servo_CW_positive);
	ssc[2].config(servo_min_duty, servo_max_duty, servo_min_degrees, servo_max_degrees, servo_2_offset, servo_CCW_positive);
	ssc[3].config(servo_min_duty, servo_max_duty, servo_min_degrees, servo_max_degrees, servo_3_offset, servo_CW_positive);
	ssc[4].config(servo_min_duty, servo_max_duty, servo_min_degrees, servo_max_degrees, servo_4_offset, servo_CCW_positive);
	ssc[5].config(servo_min_duty, servo_max_duty, servo_min_degrees, servo_max_degrees, servo_5_offset, servo_CW_positive);

	delay(1000);
	robot.set_pose({0, 0, 10, radians(0), radians(0), radians(0)});
	delay(1000);
	robot.set_pose({0, 0, 0, radians(0), radians(0), radians(0)});
	delay(1000);


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


	// Set up logger variables for debug output
	varLogger.config(Serial, ",", true);
	varLogger.append(&ssc[0].m_pos_deg,"Servo_0_command_(degrees)");
	varLogger.append(&ssc[0].m_pos_ms,"Servo_0_command_(ms)");
	varLogger.append(&servo_0_Feedback, "Servo_0_feedback");

	varLogger.append(&ssc[1].m_pos_deg,"Servo_1_command_(degrees)");
	varLogger.append(&ssc[1].m_pos_ms,"Servo_1_command_(ms)");
	varLogger.append(&servo_1_Feedback, "Servo_1_feedback");

	varLogger.append(&ssc[2].m_pos_deg,"Servo_2_command_(degrees)");
	varLogger.append(&ssc[2].m_pos_ms,"Servo_2_command_(ms)");
	varLogger.append(&servo_2_Feedback, "Servo_2_feedback");

	varLogger.append(&ssc[3].m_pos_deg,"Servo_3_command_(degrees)");
	varLogger.append(&ssc[3].m_pos_ms,"Servo_3_command_(ms)");
	varLogger.append(&servo_3_Feedback, "Servo_3_feedback");

	varLogger.append(&ssc[4].m_pos_deg,"Servo_4_command_(degrees)");
	varLogger.append(&ssc[4].m_pos_ms,"Servo_4_command_(ms)");
	varLogger.append(&servo_4_Feedback, "Servo_4_feedback");

	varLogger.append(&ssc[5].m_pos_deg,"Servo_5_command_(degrees)");
	varLogger.append(&ssc[5].m_pos_ms,"Servo_5_command_(ms)");
	varLogger.append(&servo_5_Feedback, "Servo_5_feedback");

	varLogger.logHeader();
	
	testTimer.begin(interruptTimer,5000);
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

void interruptTimer()
{
	screen.forceProcess();
	sensor.forceProcess();
	controller.forceProcess();
	inpU = controller.getU();
	robot.set_pose({0,0,0,inpU[0],inpU[1],0});
	//platformHeight = 5.0*sin(3.0*millis()/1000.0);
	
	//platformHeight = radians(10);

	// if(millis()/1000.0 > 10.0)
	// {
	// 	platformHeight = 5.0;
	// }

	// if(millis()/1000.0 > 13.0)
	// {
	// 	platformHeight = -5.0;
	// }

	//robot.set_pose({0,0,0,0,platformHeight,0});

	servo_0_Feedback = analogRead(servo_0_pin);
	servo_1_Feedback = analogRead(servo_1_pin);
	servo_2_Feedback = analogRead(servo_2_pin);
	servo_3_Feedback = analogRead(servo_3_pin);
	servo_4_Feedback = analogRead(servo_4_pin);
	servo_5_Feedback = analogRead(servo_5_pin);

	varLogger.log();
	//robot.set_pose_at_point({0,0,0,inpU[0],inpU[1],0},double(screen.getX()),double(screen.getY()));
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
