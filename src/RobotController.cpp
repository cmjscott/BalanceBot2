#include "RobotController.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

void RobotController::begin(SSC32& ssc, int servo0, int servo1, int servo2, int servo3, int servo4, int servo5)
{
	m_ssc = &ssc;
	m_servo0 = &ssc[servo0];
	m_servo1 = &ssc[servo1];
	m_servo2 = &ssc[servo2];
	m_servo3 = &ssc[servo3];
	m_servo4 = &ssc[servo4];
	m_servo5 = &ssc[servo5];

	m_biasX = 0;
	m_biasY = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void RobotController::config()
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////

void RobotController::set_pose(platform_t desiredPose)
{
	platform_t newPose = {desiredPose.hx_x, desiredPose.hx_y, desiredPose.hx_z,
												desiredPose.hx_a - m_biasX, desiredPose.hx_b - m_biasY, desiredPose.hx_c};

	m_moveOK = m_ik.calcServoAnglesOld(newPose, m_servoAngles);
	updateServos(m_moveOK);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void RobotController::set_pose_at_point(platform_t desiredPose, double x_pos, double y_pos)
{
    double z = sin(desiredPose.hx_a)*x_pos;
    double x = x_pos*(1.0-cos(desiredPose.hx_a));

	platform_t newPose = {desiredPose.hx_x, desiredPose.hx_y + x, desiredPose.hx_z + z,
												desiredPose.hx_a - m_biasX, desiredPose.hx_b - m_biasY, desiredPose.hx_c};

	m_moveOK = m_ik.calcServoAngles(newPose, m_servoAngles);
	if (m_moveOK == 0)
	{
		updateServos(m_moveOK);
	}

}

////////////////////////////////////////////////////////////////////////////////////////////////////

void RobotController::home()
{
	m_moveOK = m_ik.home(m_servoAngles);
	updateServos(m_moveOK);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void RobotController::setBias(double _biasX, double _biasY)
{
	m_biasX = _biasX;
	m_biasY = _biasY;

}

////////////////////////////////////////////////////////////////////////////////////////////////////

void RobotController::updateServos(int8_t movOK)
{
    // Statistics of errors.
    static double nbMov = 0;
    static double nbMovNOK = 0;
    static double NOKrate = 0;
    nbMov++;

    if (movOK == 0)
    {
        // Write to servos.

        for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
        {
            m_ssc->operator[](sid).set_degrees(m_servoAngles[sid].deg);
        }

			m_ssc->commit();

#if false
        // Write servo angles to Serial for debug.
        for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
        {
            //Serial.printf("%4d ", servo_angles[sid].us);
        }
        //Serial.println("");
#endif
    }
    else
    {
        // Error handling.
        nbMovNOK++;
        NOKrate = (nbMovNOK / nbMov) * (double)100.0;
        //Serial.printf("%10lu", millis());
        //Serial.printf(" | BAD MOVE | movOK = %d", movOK);
        //Serial.printf(" | NB MOV = %10.0f", nbMov);
        //Serial.printf(" | NOK rate = %4.1f %%", NOKrate);
        //Serial.print("\n");
    }
}
