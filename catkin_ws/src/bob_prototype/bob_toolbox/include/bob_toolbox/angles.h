#ifndef _BOB_TOOLBOX_ANGLES_H_
#define _BOB_TOOLBOX_ANGLES_H_

#include <cmath>

namespace bob
{

	static const float TWO_PI = 2.0f * M_PI;

	//! Returns an angle from 0 to 2pi
	static inline float normalizeAnglePos(float angle)
	{
		return fmod(fmod(angle, TWO_PI) + TWO_PI, TWO_PI);
	}

	//! Returns an angle from -pi to pi
	static inline float normalizeAngle(float angle)
	{
		float a = normalizeAnglePos(angle);
		if (a > M_PI)
		{
			a -= TWO_PI;
		}
		return a;
	}

	//! The shortest distance between two angles

	//! The returned value can be positive or negative.
	//! Negative implies clockwise rotation between the angles
	static inline float shortestAngularDistance(float from, float to)
	{
		return normalizeAngle(to - from);
	}

	//! Convert from degrees to radians
	static inline float toRadian(float degree)
	{
		return (degree * M_PI / 180.0);
	}
	
	//! Convert from radians to degrees
	static inline float toDegree(float radian)
	{
		return (radian * 180.0 / M_PI);
	}

}

#endif
