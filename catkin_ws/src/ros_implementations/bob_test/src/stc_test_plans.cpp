#include <bob_test/stc_test_plans.h>

namespace bob
{
	STCTestPlans::STCTestPlans()
	{
		//getSimpleAreaPlans();
		getComplexPlan();
	}

	size_t STCTestPlans::size() const
	{
		return plans.size();
	}

	STCPlan STCTestPlans::getPlan(const unsigned id) const
	{
		if (id < plans.size())
		{
			return plans[id];
		}
		else
		{
			STCPlan empty;
			return empty;
		}
	}

	
	void STCTestPlans::getSimpleAreaPlans()
	{
		// Generate simple plans has different width, height and orientation. 
		for(float orientation = 0; orientation <= M_PI / 4; orientation += M_PI / 4)
		{
			for(int width = 1; width < 3; ++width)
			{
				for(int height = 1; height < 3; ++height)
				{
					// Get simple rectange GridPoints
					std::vector<GridPoint> gridPoints = getSimpleGridPoints(width, height, WorldPoint(0, 0));

					// Get simple GridSet
					std::vector<GridSet> simpleArea;
					simpleArea.push_back(getGridSet(WorldPoint(0, 0), orientation, gridPoints));

					// Generate plan and add to test plans
					plans.push_back(generate(simpleArea));
				}
			}
		}
	}

	void STCTestPlans::getComplexPlan()
	{
		// Generate basic grid pattern
		std::vector<GridPoint> square = getSimpleGridPoints(1, 1, WorldPoint(0, 0));
		std::vector<GridPoint> rectangle = getSimpleGridPoints(3, 1, WorldPoint(0, 0));	
		
		// Change the way they combined, keep the rectangle, move and rotate the square.
		GridSet rectGridSet = getGridSet(WorldPoint(0, 0), 0, rectangle);
		for(float orientation = 0; orientation <= M_PI / 4; orientation += M_PI / 8)
		{
			for(float position = -0.5; position <= 0.5; position += 0.1)
			{
				std::vector<GridSet> gridSets;
				gridSets.push_back(rectGridSet);
				gridSets.push_back(getGridSet(WorldPoint(1, position), orientation, square));
				plans.push_back(generate(gridSets));
			}	
		}

	}



	std::vector<GridPoint> STCTestPlans::getSimpleGridPoints(const int width, const int height, const WorldPoint& origin)
	{
		std::vector<GridPoint> gridPoints;
		for (int i = origin.x; i < origin.x + width; ++i)
		{
			for (int j = origin.y; j < origin.y + height; ++j)
			{
				gridPoints.push_back(GridPoint(i, j));
			}
		}
		return gridPoints;
	}


	GridSet STCTestPlans::getGridSet(const WorldPoint& origin, const float orientation, const std::vector<GridPoint>& gridPoints) const 
	{
		GridSet gridSet(origin, orientation);
		for(std::vector<GridPoint>::const_iterator itr = gridPoints.begin(); itr != gridPoints.end(); itr++)
		{
			gridSet.addPoint(*itr);
		}
		
		return gridSet;
	}


	STCPlan STCTestPlans::generate(const std::vector<GridSet>& gridSets)
	{
		std::list<STCPath> paths;
		for(std::vector<GridSet>::const_iterator itr = gridSets.begin(); itr != gridSets.end(); ++itr)
		{
			paths.push_back(hamiltonian.generatePath(treeBuilder.buildTree(*itr)));
		}

		STCPlan plan = combiner.combinePaths(paths);
		return plan;
	}


}

