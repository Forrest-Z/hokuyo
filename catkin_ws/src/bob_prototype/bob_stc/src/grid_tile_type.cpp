#include <bob_stc/grid_tile_type.h>

#include <set>
#include <bob_toolbox/direction.h>

namespace bob
{

const std::set<Direction> GridTileType::possibleConnections() const
{
	static std::set<Direction> toReturn;
	if (toReturn.empty())
	{
		toReturn.insert(right);
		toReturn.insert(up);
		toReturn.insert(left);
		toReturn.insert(down);
	}
	return toReturn;
}

const std::set<Direction> Horizontal::possibleConnections() const
{
	static std::set<Direction> toReturn;
	if (toReturn.empty())
	{
		toReturn.insert(left);
		toReturn.insert(right);
	}
	return toReturn;
}

const std::set<Direction> Vertical::possibleConnections() const
{
	static std::set<Direction> toReturn;
	if (toReturn.empty())
	{
		toReturn.insert(up);
		toReturn.insert(down);
	}
	return toReturn;
}

}

