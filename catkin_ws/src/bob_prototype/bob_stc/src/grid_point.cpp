#include <bob_stc/grid_point.h>

#include <boost/functional/hash/hash.hpp>

#include <bob_toolbox/direction.h>
#include <cstddef>

namespace bob 
{

	GridPoint::GridPoint(float x, float y) :
	x(x),
	y(y) {}


	std::vector<GridPoint> fourNeighbors(GridPoint point)
	{
		std::vector<Direction> directions = allDirections();
		std::vector<GridPoint> neighbors;
		for (std::vector<Direction>::iterator directionItr = directions.begin(); directionItr != directions.end(); ++directionItr)
			neighbors.push_back(pointInDirection(point, *directionItr));
		return neighbors;
	}

	bool operator==(const GridPoint& first, const GridPoint& second)
	{
		return (first.x == second.x) && (first.y == second.y);
	}

	bool operator!=(const GridPoint& first, const GridPoint& second)
	{
		return !(first == second);
	}

	std::size_t GridPointHash::operator()(const GridPoint& point) const
	{
		std::size_t seed = 0;
		boost::hash_combine(seed, point.x);
		boost::hash_combine(seed, point.y);
		return seed;
	}

}
