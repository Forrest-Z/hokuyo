#ifndef _BOB_TOOLBOX_NEWTONIAN_EQUATIONS_H_
#define _BOB_TOOLBOX_NEWTONIAN_EQUATIONS_H_

#include <cmath>

namespace bob
{

	//! Calculates the initial speed required if an object is to reach a final speed while travelling
	//! a certain distance while experiencing a constant acceleration.

	//! Based on Newton's equations of motion
	inline float initialSpeedForDistance(float acceleration, float distance, float finalSpeed=0.0)
	{
		return sqrt(pow(finalSpeed, 2) - 2 * acceleration * distance);
	}

	//! The intial speed required for an object to stop at a certain distance
	//! while experiencing a constant decelleration.
 	
	//! This is just a convienience class that delegates to initialSpeedForDistance(...)
	inline float stoppingSpeedForDistance(float decceleration, float distance)
	{
		return initialSpeedForDistance(-decceleration, distance);
	}

}

#endif
