#include <bob_stc/grid.h>

#include <utility>
#include <cmath>

#include <bob_stc/grid_point.h>
#include <bob_toolbox/world_point.h>

namespace bob 
{

	WorldPoint Grid::gridToWorld(GridPoint gridIndices) const
	{
		return gridToWorld(std::make_pair(gridIndices.x, gridIndices.y));
	}

	// Performs a simple transform to get from grid co-ordinates to world co-ordintets
	WorldPoint Grid::gridToWorld(std::pair<float, float> gridIndices) const
	{
		return gridToWorld(gridIndices.first, gridIndices.second);
	}

	WorldPoint Grid::gridToWorld(float gridX, float gridY) const
	{
		float x = origin.x + width * (cos(rotation) * gridX - sin(rotation) * gridY); 
		float y = origin.y + width * (sin(rotation) * gridX + cos(rotation) * gridY);
		WorldPoint toReturn(x,y);
		return toReturn;
	}

	GridPoint Grid::worldToGrid(WorldPoint worldCoordinates) const
	{
		// Just reverse the gridToWorld function (invert rotation matrix, etc.)
		float xOffset = worldCoordinates.x - origin.x;
		float yOffset = worldCoordinates.y - origin.y;
		float gridX = (cos(rotation) * xOffset + sin(rotation) * yOffset) / width;
		float gridY = (-sin(rotation) * xOffset + cos(rotation) * yOffset) / width;	

		return GridPoint((int)round(gridX), (int)round(gridY));
	}

	float Grid::getRotation() const
	{
		return rotation;
	}

	float Grid::getWidth() const
	{
		return width;
	}

	WorldPoint Grid::getOrigin() const
	{
		return origin;
	}

}
