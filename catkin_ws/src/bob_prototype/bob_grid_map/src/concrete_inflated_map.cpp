#include <bob_grid_map/concrete_inflated_map.h>
#include <bob_toolbox/easy_print.h>
#include <limits>
#include <bob_config/config.h>
#include <bob_grid_map/iobstacle_map.h>

namespace bob
{

	ConcreteInflatedMap::ConcreteInflatedMap(float resolution) : 
		LocationMapper(resolution),
		distanceCacheMatrix(Config::OBSTACLE_INFLATION_DISTANCE, resolution)

	{}

	void ConcreteInflatedMap::setInflationData(const IObstacleMap& map)
	{
		// Clear costmaps so that they are fresh
		MapMetadata masterMetadata = map.getMapMetadata();
		resizeClear(masterMetadata);
		closedSet.resizeClear(masterMetadata);

		// Add the obstacles from the map into the queue
		enqueueObstacles(map);

		expandQueueUntilEmpty();
	}

	std::vector<IntVector> ConcreteInflatedMap::vectorsInAllDirections()
	{
		std::vector<IntVector> toReturn;
		toReturn.push_back(IntVector(1, 0));
		toReturn.push_back(IntVector(0, 1));
		toReturn.push_back(IntVector(-1, 0));
		toReturn.push_back(IntVector(0, -1));
		return toReturn;
	}

	void ConcreteInflatedMap::expandNextElement()
	{
		std::vector<IntVector> vectors = vectorsInAllDirections();

		// Get the cell with the smallest distance
		CellData current_cell = inflationQueue.top();
		inflationQueue.pop();

		IntPoint oldPoint = current_cell.point;
		IntPoint closestObstacle = current_cell.closestObstacle;

		// Attempt to put the neighbors of the current cell onto the queue
		for (auto vectorItr = vectors.begin(); vectorItr != vectors.end(); ++vectorItr)
		{
			IntPoint neighbor = oldPoint + *vectorItr;
			if (isInside(neighbor) && !closedSet[neighbor])
			{
				enqueue(neighbor, closestObstacle);
			}
		}

	}

	void ConcreteInflatedMap::expandQueueUntilEmpty()
	{
		while (!inflationQueue.empty())
		{
			expandNextElement();
		}
	}

	void ConcreteInflatedMap::enqueueObstacles(const IObstacleMap& sourceMap)
	{
		MapMetadata metadata = sourceMap.getMapMetadata();
		MapLocation bottomLeftCorner = metadata.bottomLeftCorner;
		MapLocation upperRightCorner = metadata.upperRightCorner();

		for (int x = bottomLeftCorner.x; x <= upperRightCorner.x; x++)
		{
			for (int y = bottomLeftCorner.y; y <= upperRightCorner.y; y++)
			{
				IntPoint point(x, y);
				FreeState pointState = sourceMap.pointFree(point);
				if (pointState == SensedObstacle ||
					pointState == HiddenObstacle)
				{
					enqueue(point, point);
				}
			}
		}
	}

	void ConcreteInflatedMap::enqueue(IntPoint point, IntPoint closestObstacle)
	{
		// Push the cell data onto the queue and mark
		closedSet[point] = true;

		// We compute our distance table one cell further than the inflation radius dictates so we can make the check below
		float distance;
		if (!distanceCacheMatrix.distanceLookup(point - closestObstacle, distance))
			return;

		(*this)[point] = distance;

		CellData queuePoint(distance, point, closestObstacle);
		inflationQueue.push(queuePoint);
	}

	float ConcreteInflatedMap::defaultCellValue() const
	{
		return std::numeric_limits<float>::max();
	}

	float ConcreteInflatedMap::obstacleDistance(MapLocation location) const
	{
		return getValue(location);
	}

} 
