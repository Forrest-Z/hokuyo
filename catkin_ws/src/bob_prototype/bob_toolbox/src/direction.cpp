#include <bob_toolbox/direction.h>
#include <set>

namespace bob
{
	
	std::set<Direction> allDirectionsSet()
	{
		std::set<Direction> toReturn;
		toReturn.insert(left);
		toReturn.insert(down);
		toReturn.insert(right);
		toReturn.insert(up);
		return toReturn;
	}	
	std::vector<Direction> allDirections()
	{
		std::vector<Direction> toReturn;
		toReturn.push_back(left);
		toReturn.push_back(down);
		toReturn.push_back(right);
		toReturn.push_back(up);
		return toReturn;
	}	

	Direction nextDirection(const Direction& direction, bool clockwise)
	{
		int moveAmount = clockwise ? 3 : 1;
		return rotateDirection(direction, moveAmount);
	}

	Direction rotateDirection(const Direction& direction, int amount)
	{
		return (Direction)(((int)direction + amount) % 4);		
	}
}
