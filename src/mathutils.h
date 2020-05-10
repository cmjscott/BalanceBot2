#ifndef __MATHUTILS_H__
#define __MATHUTILS_H__


float periodicmod(float x, float y);

float inrange(float x, float min, float max);

float saturate(float x, float min, float max);

float squash(float x, float lBound, float hBound);

float sign(float x);

float scale(float x, float from_lBound, float from_hBound, float to_lBound, float to_hBound);

#endif // __MATHUTILS_H__
