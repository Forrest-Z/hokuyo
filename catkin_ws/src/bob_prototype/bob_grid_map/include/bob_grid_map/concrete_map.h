#ifndef _BOB_GRID_MAP_CONCRETE_MAP_H_
#define _BOB_GRID_MAP_CONCRETE_MAP_H_

#include <bob_grid_map/decay_probability_cell.h>
#include <bob_grid_map/map_location_set.h>

#include <bob_grid_map/raw_map.h>
#include <bob_grid_map/iobstacle_map.h>
#include <bob_grid_map/iprobability_map.h>
#include <bob_grid_map/ihidden_obstacle_map.h>

#include <bob_toolbox/bounds2d.h>

namespace bob 
{


	template <typename ProbabilityCell>
		class ConcreteMap : public RawMap<ProbabilityCell>, public IObstacleMap, public IProbabilityMap, public IHiddenObstacleMap
	{

		public:

			ConcreteMap(float resolution) :
				LocationMapper(resolution)
		{}

			virtual FreeState pointFree(MapLocation point) const
			{
				FreeState sensedState = RawMap<ProbabilityCell>::getValue(point).getFreeState();
				if (sensedState == Free && hiddenObstacles.count(point) == 1)
				{
					return HiddenObstacle;
				}
				return sensedState;
			}

			virtual void setObstacle(MapLocation point)
			{
				(*this)[point].update(true);
			}

			virtual void setFree(MapLocation point)
			{
				(*this)[point].update(false);
			}

			virtual void expandToIncludeBounds(Bounds2D<float> bounds)
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

			virtual IProbabilityMap::data_type getProbability(MapLocation point) const
			{
				return RawMap<ProbabilityCell>::getValue(point).getProbability();
			}

			FreeState stateFromProbability(float probability) const
			{
				if (probability < 0)
					return Unknown;
				else if (probability > 0.25)
					return SensedObstacle;
				else
					return Free;			
			}

			virtual void setValue(MapLocation point, float probability)
			{
				if (probability == -1)
					(*this)[point] = defaultCellValue();
				else
					(*this)[point] = ProbabilityCell(probability);
			}

			virtual void addNewHiddenObstacle(const WorldPoint& obstacle)
			{
				MapLocation location = worldToMap(obstacle);
				hiddenObstacles.insert(location);
			}

		private:

			virtual ProbabilityCell defaultCellValue() const 
			{
				return ProbabilityCell();
			}

			MapLocationSet hiddenObstacles;

	};

}

#endif
