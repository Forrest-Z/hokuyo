#ifndef _BOB_TOOLBOX_MAP_HALF_SPACE_H_
#define _BOB_TOOLBOX_MAP_HALF_SPACE_H_

#include <bob_toolbox/world_point.h>

namespace bob
{

	//! This class represents the division of a map into two sections, along a line.
	//! Search google for "half-space" to obtain some intuition about the topic.
	class MapHalfSpace
	{

		public:

			//! Generates a normal vector from an angle
			MapHalfSpace(WorldPoint origin, float normalAngle);

			//! Tests to see if a point lies within the half space
			bool contains(WorldPoint point) const;

		private:

			//! A point that lies on the border of the half-space
			WorldPoint origin;

			//! These floats represent a vector normal to the origin, pointing into the
			//! half space.
			WorldVector normal;

	};

}

#endif
