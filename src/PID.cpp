#include <Arduino.h>

#include "PID.h"
#include "mathutils.h"


float PID::compute(float setpoint, float input, float timestep)
{
	float currentError = setpoint - input;

	//Hard coaded gains, like some sort of professional...
	//TODO: set these values properly.
	float Kc 	= 0.34321;
	float N 	= 0.08784;
	float D		= 0.6869;

	//D = .58;

	float A = m_Kp * timestep + m_Kd ;
	float B = -(m_Kd + N * m_Kp * timestep + N * m_Kd);
	float C = m_Kd * N;

	float output = Kc / timestep * (A * currentError + B * m_error_km1 + C * m_error_km2) + D * m_output_km1;

	m_error_km2 = m_error_km1;
	m_error_km1 = currentError;
	m_output_km1 = output;

	return saturate(output, m_minOutput, m_maxOutput);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void PID::reset()
{
	m_errorIntegral = 0;
	m_previousError = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
