#ifndef _BOB_TOOLBOX_VELOCITY2D_H_
#define _BOB_TOOLBOX_VELOCITY2D_H_

namespace bob
{

	//! Represents a description of the robot's velocity
	struct Velocity2D
	{
		//! Constructor which initializes values
		Velocity2D(float x, float w) : 
		x(x), 
		w(w) 
		{}

		//! Empty constructor, init to zero
		Velocity2D() : x(0), w(0) {}

		//! Linear velocity
		float x;

		//! Rotational velocity
		float w;

		//! Inequality operator
		bool operator!=(const Velocity2D& toCompare) const
		{
			return (x != toCompare.x || w != toCompare.w);	
		}

		//! Equality operator
		bool operator==(const Velocity2D& toCompare) const
		{
			return	(x == toCompare.x && w == toCompare.w);
		}
	};

}

#endif

