#ifndef _BOB_TOOLBOX_LINE_SEGMENT_H_
#define _BOB_TOOLBOX_LINE_SEGMENT_H_

#include <bob_toolbox/world_point.h>
#include <bob_toolbox/geometry.h>

namespace bob
{

	struct LineSegment	
	{
		LineSegment(WorldPoint first, WorldPoint second) :
			first(first),
			second(second)
		{}

		LineSegment() {}

		WorldPoint first;
		WorldPoint second;

		float length()
		{
			return diagonalDistance(first, second);
		}

		bool operator==(const LineSegment& rhs) const
		{
			return ((first == rhs.first) && (second == rhs.second)) ||
				((second == rhs.first) && (first == rhs.second));
		}
		
	};

}

#endif
