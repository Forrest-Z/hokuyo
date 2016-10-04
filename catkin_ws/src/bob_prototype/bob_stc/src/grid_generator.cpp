#include <bob_stc/grid_generator.h>

#include <limits>
#include <bob_grid_map/map_location_set.h>
#include <bob_toolbox/world_point.h>
#include <bob_coverage/area_tools.h>
#include <bob_grid_map/raw_map.h>
#include <bob_toolbox/world_point_shapes.h>
#include <iostream>
#include <bob_toolbox/grid_algorithms.h>
#include <bob_coverage/discrete_area_shapes.h>
#include <boost/bind.hpp>

namespace bob
{

	GridSet GridGenerator::optimalSet(const DiscreteArea& areaToCover, DiscreteArea& coveredArea) const
	{
		std::vector<WorldPoint> origins = gridOriginsToTry();
		std::vector<float> angles = anglesToTry();

		// Combination of origin and angle
		std::pair<WorldPoint, float> bestCombination;
		int smallestSize = std::numeric_limits<int>::max();

		for (std::vector<float>::iterator angleItr = angles.begin(); angleItr != angles.end(); ++angleItr)
		{ 
			for (std::vector<WorldPoint>::iterator originItr = origins.begin(); originItr != origins.end(); ++originItr)
			{
				// Create a gridmap of free cells in the costmap
				GridSet set = generateSet(areaToCover, *originItr, *angleItr, coveredArea);

				// Get the areaToCover
				DiscreteArea uncoveredArea = areaToCover;
				
				// Check how much area is not covered
				uncoveredArea.erase(coveredArea);
				
				if(uncoveredArea.size() == 0)
                		{
					// If obtained coveredArea completely covered the required areaToCover 
					// then we already have the best case
                    			return set;
                		}
				else if(uncoveredArea.size() < smallestSize)
                		{
					// Accept if better than previous best
                    			smallestSize = uncoveredArea.size();
                    			bestCombination.first = *originItr;
                    			bestCombination.second = *angleItr;
                		}
				
			}
		}

		// Regenerate the best combination to return it
		return generateSet(areaToCover, bestCombination.first, bestCombination.second, coveredArea);
	}

	std::vector<WorldPoint> GridGenerator::gridOriginsToTry() const
	{
		std::vector<WorldPoint> origins;

		int numberOfShifts = Config::STC_NUM_SHIFTS;
		
		// Calculate the origins that will be tested
		float shiftFactor = gridWidth / numberOfShifts;
		for (int shiftXIndex = 0; shiftXIndex < numberOfShifts; shiftXIndex++)
		{
			for (int shiftYIndex = 0; shiftYIndex < numberOfShifts; shiftYIndex++)
			{
				origins.push_back(WorldPoint(shiftFactor * shiftXIndex, shiftFactor * shiftYIndex));
			}
		}
		
		return origins;
	}	

	std::vector<float> GridGenerator::anglesToTry() const
	{
		std::vector<float> angles;
		
		int numberOfAngles = Config::STC_NUM_ANGLES;

		// Only do a quarter of full rotation, due to rotational symmetry
		float angleFactor = (M_PI / 2) / numberOfAngles;	

		for (int angleIndex = 0; angleIndex < numberOfAngles; angleIndex++)
		{ 
			float angle = angleIndex * angleFactor;
			angles.push_back(angle);
		}
		return angles;
	}

	GridSet GridGenerator::generateSet(DiscreteArea areaToCover, WorldPoint origin, float rotation, DiscreteArea& coveredArea) const
	{
		GridSet toReturn(origin, rotation, gridWidth);

        	// Clear the Discrete Cells from the coveredArea
		coveredArea.clear();

		while (!areaToCover.empty())
		{
			// Get an arbitrary point from areaToCover
			MapLocation topPoint = areaToCover.arbitraryCell();
			areaToCover.erase(topPoint);

			// Get the GridPoint that the mapLocation corresponds to
			WorldPoint topWorldPoint = areaToCover.mapToWorld(topPoint);
			GridPoint newGrid = toReturn.worldToGrid(topWorldPoint);

			// Calculate the area which will be covered by this gridPoint
			WorldPoint centerOfGridPoint = toReturn.gridToWorld(newGrid);
			DiscreteArea gridArea = discreteAreaSquare(centerOfGridPoint, rotation, coverageWidth, areaToCover.getResolution());	

			bool freeOfObstacles = true;
			for (MapLocationSet::iterator cellItr = gridArea.begin(); cellItr != gridArea.end(); ++cellItr)
			{
				// Remove points which will cause this grid point to come up again
				if (areaToCover.contains(*cellItr) && toReturn.worldToGrid(areaToCover.mapToWorld(*cellItr)) == newGrid)
				{
					areaToCover.erase(*cellItr);
				}
			}

			for (MapLocationSet::iterator cellItr = gridArea.begin(); cellItr != gridArea.end(); ++cellItr)
			{
				// If the buffered portion hits a wall, then do not accept the grid point
                		WorldPoint cellWorldPoint = gridArea.mapToWorld(*cellItr);
				MapLocation location = obstacleMap.worldToMap(cellWorldPoint);
				if (obstacleMap.pointFree(location) == SensedObstacle)
				{
					freeOfObstacles = false;
					break;
				}
			}

			if (freeOfObstacles)
			{
				toReturn.addPoint(newGrid);
				coveredArea.insert(gridArea);	
			}

		}
		return toReturn;
	}
}
