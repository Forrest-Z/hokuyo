#ifndef _BOB_TOOLBOX_DIRECTION_H_
#define _BOB_TOOLBOX_DIRECTION_H_

#include <vector>
#include <set>

namespace bob 
{

	//! Enum used to represent directions on a grid
	enum Direction 
	{ 
		left = 0, 
		down = 1, 
		right = 2, 
		up = 3, 
		invalid 
	};

	std::vector<Direction> allDirections();
	std::set<Direction> allDirectionsSet();
	
	//! Gives the next direction encountered when travelling clockwise or counter-clockwise

	//! Input:
	//! lastDirection - The previous direction
	//! clockwise - If True, travels clockwise to find the next direction		
	Direction nextDirection(const Direction& direction, bool clockwise=false);

	//! Rotates a direction counter-clockwise by the given amount)
	Direction rotateDirection(const Direction& direction, int amount);

	//! Gets a point in the direction specified by Direction
	template <typename PointType>
	PointType pointInDirection(PointType point, Direction direction)
	{
		PointType newPoint;
		switch (direction)
		{	
			case right:
				newPoint.x = point.x + 1;
				newPoint.y = point.y;
				break;

			case up:
				newPoint.x = point.x;
				newPoint.y = point.y + 1;
				break;		

			case left:
				newPoint.x = point.x - 1;
				newPoint.y = point.y;
				break;		
			
			case down:
				newPoint.x = point.x;
				newPoint.y = point.y - 1;
				break;
		}
		return newPoint;
	}

	//! Calculate the Direction
	template <typename PointType>
	Direction directionBetween(const PointType& from, const PointType& to)
	{
		int xDiff = to.x - from.x;
		int yDiff = to.y - from.y;
		
		if (xDiff == 0 && yDiff > 0)
			return up;
		else if (xDiff > 0 && yDiff == 0)
			return right;
		else if (xDiff == 0 && yDiff < 0)
			return down;
		else if (xDiff < 0 && yDiff == 0)
			return left;
		else
			return invalid;
	}
}

#endif
