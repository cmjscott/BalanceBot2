#include "SSC32.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

void SSC32::Servo::attach(SSC32* ssc, int no)
{
	m_ssc = ssc;
	m_no = no;
	m_min = 500;
	m_pos = 1500;
	m_max = 2500;
	m_deg_min = -90;
	m_deg_max = 90;
	m_offset = 0;
	m_is_changed = false;
	m_cfg_CCW = false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

int SSC32::Servo::get_position()
{
	return m_pos;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void SSC32::Servo::set_position(int ms)
{
	m_pos = constrain(ms, m_min, m_max);
	//Serial.print(m_no);
	//Serial.println(m_pos);
	m_is_changed = true;
	m_ssc->servo_on_changed();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

float SSC32::Servo::get_degrees()
{
	float deg = (m_pos - m_min) * (m_deg_max - m_deg_min) / (m_max - m_min) + m_deg_min;

	if (m_cfg_CCW)
	{
		deg = m_deg_max - deg;
	}

	return deg;
	//return (m_pos - m_min) * (m_deg_max - m_deg_min) / (m_max - m_min) + m_deg_min;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void SSC32::Servo::set_degrees(double degrees)
{
	float deg;
	if (m_cfg_CCW)
	{
		deg = m_deg_max - degrees;
	} else {
		deg = degrees;
	}
	int ms = (deg - m_deg_min) * (m_max - m_min) / (m_deg_max - m_deg_min) + m_min;
	ms = ms + m_offset;

	m_pos_deg = deg;
	m_pos_ms = ms;
	set_position(ms);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

float SSC32::Servo::get_radians()
{
	return radians(get_degrees());
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void SSC32::Servo::set_radians(float radians)
{
	set_degrees(degrees(radians));
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void SSC32::Servo::config(int min, int max, float deg_min, float deg_max, int offset, bool cfg_CCW)
{
	m_min = min;
	m_max = max;
	m_deg_min = deg_min;
	m_deg_max = deg_max;
	m_offset = offset;
	m_cfg_CCW = cfg_CCW;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void SSC32::Servo::send_cmd_string()
{
	if (m_is_changed)
	{
		m_is_changed = false;
		m_ssc->m_ser->write('#');
		m_ssc->m_ser->print(m_no);
		m_ssc->m_ser->write('P');
		m_ssc->m_ser->print(m_pos);

		//Debug
		//Serial.write("#");
		//Serial.print(m_no);
		//Serial.write("P");
		//Serial.print(m_no);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void SSC32::begin(Stream& ser, unsigned long autocommit)
{
	m_ser = &ser;
	m_autocommit = autocommit;
	for (int i = 0; i < 32; i++)
		m_servos[i].attach(this, i);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

SSC32::Servo& SSC32::operator[](int i)
{
	return m_servos[i];
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void SSC32::servo_on_changed()
{
	if (m_autocommit != (unsigned long)(-1))
		commit(m_autocommit);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void SSC32::commit(unsigned long time)
{
	bool cmd = false;
	for (int i = 0; i < 32; i++)
	{
		cmd |= m_servos[i].m_is_changed;
		m_servos[i].send_cmd_string();
	}
	if (time != (unsigned long)(-1) && cmd)
	{
		m_ser->write('T');
		m_ser->print(time);
	}
	m_ser->write('\r');
	//Serial.write('\n');
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool SSC32::is_done()
{
	while (m_ser->available())
		m_ser->read();
	m_ser->write("Q\r");
	while (!m_ser->available());
	char r = m_ser->read();
	return r == '.';
}

////////////////////////////////////////////////////////////////////////////////////////////////////
