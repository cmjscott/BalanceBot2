#ifndef __ROBOTCONTROLLER_H__
#define __ROBOTCONTROLLER_H__

#include "SSC32.h"
#include "Hexapod_Config_1.h"
#include "Hexapod_Kinematics.h"


class RobotController
{
public:
	Hexapod_Kinematics m_ik;

	void begin(SSC32& ssc, int servo0, int servo1, int servo2, int servo3, int servo4, int servo5);

	void config();

	void set_pose(platform_t desiredPose);

	void set_pose_at_point(platform_t desiredPose, double x_pos = 0, double y_pos = 0);

	void home();

	void setBias(double _biasX, double _biasY);


private:

	void updateServos(int8_t moveOK);

	SSC32 *m_ssc;
	SSC32::Servo *m_servo0, *m_servo1, *m_servo2, *m_servo3, *m_servo4, *m_servo5;


	int8_t m_moveOK;
	angle_t m_servoAngles[NB_SERVOS];

	double m_biasX, m_biasY;


	friend void setup();
	friend void loop();
};

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif // __ROBOTCONTROLLER_H__
