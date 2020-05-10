#ifndef __SSC32_H__
#define __SSC32_H__

#include <Arduino.h>

////////////////////////////////////////////////////////////////////////////////////////////////////

class SSC32
{
public:

////////////////////////////////////////////////////////////////////////////////////////////////////

	class Servo
	{
	public:

		int get_position();
		void set_position(int ms);

		float get_degrees();
		void set_degrees(double degrees);

		float get_radians();
		void set_radians(float radians);

		void config(int min, int max, float deg_min, float deg_max, int offset, bool cfg_CCW);

	private:
		friend class SSC32;

		void attach(SSC32* ssc, int no);

		void send_cmd_string();

		SSC32* m_ssc;
		int m_no;
		int m_min;
    int m_pos;
    int m_max;
		int m_offset;
    float m_deg_max;
    float m_deg_min;
    bool m_is_changed;
		bool m_cfg_CCW;
	};

////////////////////////////////////////////////////////////////////////////////////////////////////

	void begin(Stream& ser, unsigned long autocommit=-1);

	Servo& operator[](int i);

	void commit(unsigned long time=-1);

	bool is_done();

private:
	friend class Servo;

	void servo_on_changed();

	Stream* m_ser;
	Servo m_servos[32];
	unsigned long m_autocommit;
};

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif // __SSC32_H__
