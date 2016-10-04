#include <bob_grid_map/concrete_map.h>

#include <bob_toolbox/easy_print.h>

namespace bob 
{

	/*
	FreeState ConcreteMap::stateFromProbability(float probability) const
	{
		if (probability < 0)
			return Unknown;
		else if (probability > 0.25)
			return Obstacle;
		else
			return Free;			
	}

	FreeState ConcreteMap::pointFree(MapLocation point) const
	{
		return RawMap<ProbabilityCell>::getValue(point).getFreeState();
	}

	void ConcreteMap::setObstacle(MapLocation point)
	{
		(*this)[point].update(true);
	}

	void ConcreteMap::setFree(MapLocation point)
	{
		(*this)[point].update(false);
	}

	void ConcreteMap::expandToIncludeBounds(Bounds2D<float> bounds)
	{
		WorldPoint bottomLeftPoint(bounds.minX, bounds.minY);
		WorldPoint upperRightPoint(bounds.maxX, bounds.maxY);

		MapLocation inputBottomLeft = worldToMap(bottomLeftPoint);
		MapLocation inputUpperRight = worldToMap(upperRightPoint);

		MapMetadata oldMetadata = getMapMetadata();

		// We only have to check two opposite corners to see if resizing is necessary
		if (!RawMap<ProbabilityCell>::isInside(inputBottomLeft) || !RawMap<ProbabilityCell>::isInside(inputUpperRight))
		{
			MapLocation oldBottomLeft = oldMetadata.bottomLeftCorner;
			MapLocation oldUpperRight = oldMetadata.upperRightCorner();

			// Calculating new bottom left location
			MapLocation newBottomLeft;
			newBottomLeft.x = std::min(oldBottomLeft.x, inputBottomLeft.x);
			newBottomLeft.y = std::min(oldBottomLeft.y, inputBottomLeft.y);

			// Calculating new bottom left location
			MapLocation newUpperRight;
			newUpperRight.x = std::max(oldUpperRight.x, inputUpperRight.x);
			newUpperRight.y = std::max(oldUpperRight.y, inputUpperRight.y);

			// Calculating new width and height
			int newWidth =  std::max(oldMetadata.bounds.width,  newUpperRight.x - newBottomLeft.x + 1);
			int newHeight = std::max(oldMetadata.bounds.height, newUpperRight.y - newBottomLeft.y + 1);

			// Pack the data
			MapMetadata newShape;
			newShape.bounds = Dimension2D<int>(newWidth, newHeight);
			newShape.bottomLeftCorner = newBottomLeft;

			// Note: we do not clear the map when resizing!
			resize(newShape);
		}
	}

	IProbabilityMap::data_type ConcreteMap::getProbability(MapLocation point) const
	{
		return RawMap<ProbabilityCell>::getValue(point).getProbability();
	}

	void ConcreteMap::setValue(MapLocation point, float probability)
	{
		if (probability == -1)
			(*this)[point] = defaultCellValue();
		else
			(*this)[point] = ProbabilityCell(probability);
	}
	*/
}
