#ifndef _BOB_TOOLBOX_LINE_H_
#define _BOB_TOOLBOX_LINE_H_

#include <bob_toolbox/world_point.h>
#include <iostream>

#include <cmath>

namespace bob
{

	struct Line
	{

		Line(WorldPoint origin, WorldVector vector) :
			origin(origin),
			vector(vector)
		{}

		explicit Line(WorldPoint origin=WorldPoint(), float angle=0) :
			origin(origin),
			vector(cos(angle), sin(angle))
		{}

		WorldPoint origin;
		WorldVector vector;	

		float getAngle() const
		{
			return atan2(vector.y, vector.x);
		}

		//! Calculates intersection of two lines using basic linear algebra
		//! This function attempts to solve for p = origin1 + k1 * v1 = origin2 + k2 * v2
		bool intersection(Line other, WorldPoint& returnVal) const
		{
			// Determinate of [v1  -v2]
			float determinate = -vector.x * other.vector.y + vector.y * other.vector.x;

			// No intersection exists
			if (determinate == 0)
				return false;
			
			WorldVector originDiff = other.origin - origin;

			// Motivated by: [k1   k2]' = inv([v1   -v2]) * (origin2 - origin1)
			// The constant is k1
			float constant = (originDiff.x * -other.vector.y + originDiff.y * other.vector.x) / determinate;

			// Calculating the intersection
			returnVal = origin + constant * vector;
			return true;
		}
	};
}

#endif
