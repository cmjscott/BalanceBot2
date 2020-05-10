#ifndef __QUATERNION_H__
#define __QUATERNION_H__

#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////////////////////////

struct Quaternion
{
	float w, x, y, z;

	inline Quaternion operator+(const Quaternion& other) const;
	inline Quaternion operator-(const Quaternion& other) const;
	inline Quaternion operator*(const Quaternion& other) const;
	
	inline Quaternion operator*(float real) const;
	friend inline Quaternion operator*(float real, const Quaternion& q);

	inline Quaternion operator/(float real) const;

	inline float norm() const;

	inline Quaternion conjugate() const;
	inline Quaternion normalized() const;

	friend inline Quaternion lerp(const Quaternion& q0, const Quaternion& q1, float t);
	friend inline Quaternion slerp(const Quaternion& q0, const Quaternion& q1, float t);

	inline float get_roll() const;
	inline float get_pitch() const;
	inline float get_yaw() const;
};

////////////////////////////////////////////////////////////////////////////////////////////////////

Quaternion Quaternion::operator+(const Quaternion& other) const
{
	return
	{
		w + other.w,
		x + other.x,
		y + other.y,
		z + other.z
	};
}

Quaternion Quaternion::operator-(const Quaternion& other) const
{
	return
	{
		w - other.w,
		x - other.x,
		y - other.y,
		z - other.z
	};
}

Quaternion Quaternion::operator*(const Quaternion& other) const
{
	return
	{
		w * other.w - x * other.x - y * other.y - z * other.z,
		w * other.x + x * other.w + y * other.z - z * other.y,
		w * other.y - x * other.z + y * other.w + z * other.x,
		w * other.z + x * other.y - y * other.x + z * other.w
	};
}

Quaternion Quaternion::operator*(float real) const
{
	return
	{
		w * real,
		x * real,
		y * real,
		z * real
	};
}

Quaternion operator*(float real, const Quaternion& q)
{
	return q * real;
}

Quaternion Quaternion::operator/(float real) const
{
	return
	{
		w / real,
		x / real,
		y / real,
		z / real
	};
}

////////////////////////////////////////////////////////////////////////////////////////////////////

float Quaternion::norm() const
{
	const float squared_norm = w * w + x * x + y * y + z * z;
	return sqrt(squared_norm);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

Quaternion Quaternion::conjugate() const
{
	return {+w, -x, -y, -z};
}

Quaternion Quaternion::normalized() const
{
	// Implements the Quake III Arena Fast Inverse Square Root algorithm
	// See https://en.wikipedia.org/wiki/Fast_inverse_square_root
	const float squared_norm = w * w + x * x + y * y + z * z;
	union {float f; uint32_t i;} y;
	y.f = squared_norm;
	y.i = 0x5f3759df - (y.i >> 1);
	y.f = y.f * (1.5f - (0.5f * squared_norm * y.f * y.f));
//	y.f = y.f * (1.5f - (0.5f * squared_norm * y.f * y.f));
	return operator*(y.f);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

Quaternion lerp(const Quaternion& q0, const Quaternion& q1, float t)
{
	return (1 - t) * q0 + t * q1;
}

Quaternion slerp(const Quaternion& q0, const Quaternion& q1, float t)
{
	const double omega = acos(q0.w * q1.w + q0.x * q1.x + q0.y * q1.y + q0.z * q1.z);
	if (abs(omega) > 1e6) // Prevents numerical stability issues
		return (sin((1 - t) * omega) * q0 + sin(t * omega) * q1) / sin(omega);
	else
		return lerp(q0, q1, t);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

float Quaternion::get_roll() const
{
	const float t0 = 2 * (w * x + y * z);
	const float t1 = 1 - 2 * (x * x + y * y);
	return atan2(t0, t1);
}

float Quaternion::get_pitch() const
{
	const float t2 = 2 * (w * y - z * x);
	return asin((t2 < -1) ? -1 : (t2 > 1) ? 1 : t2);
}

float Quaternion::get_yaw() const
{
	const float t3 = 2 * (w * z + x * y);
	const float t4 = 1 - 2 * (y * y + z * z);  
	return atan2(t3, t4);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif // __QUATERNION_H__
