#ifndef _BOB_STC_GRID_POINT_H_
#define _BOB_STC_GRID_POINT_H_

#include <bob_toolbox/direction.h>
#include <cstddef>

namespace bob 
{

	enum GridPointType
	{
		CompletelyFree,
		PartiallyFree,
		NotFree
	};
	
	//! Represents a point in a grid
	struct GridPoint
	{
		float x;
		float y;

		GridPoint(float x=0, float y=0);
	};

	//! These functions allow GridPoint to be stored in a std::unordered_set
	bool operator==(const GridPoint& first, const GridPoint& second);
	bool operator!=(const GridPoint& first, const GridPoint& second);

	class GridPointHash
	{

		public:
	
			std::size_t operator()(const GridPoint& point) const;
	
	};

}

#endif
