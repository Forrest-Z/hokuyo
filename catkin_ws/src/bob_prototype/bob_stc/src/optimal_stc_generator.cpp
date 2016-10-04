#include <bob_stc/optimal_stc_generator.h>

#include <bob_stc/spanning_tree_grid.h>
#include <bob_stc/stc_hamiltonian.h>
#include <bob_stc/low_bend_tree_builder.h>

namespace bob
{

	std::vector<STCPath> OptimalSTCGenerator::tryToCoverArea(const DiscreteArea& areaToCover, DiscreteArea& areaCovered) const
	{
		std::vector<STCPath> plans;

		// We don't want to generate a grid for areas which aren't connected. Result will likely be bad
		// Therefore, we separate them and generate grids for each individually
		std::vector<DiscreteArea> isolatedAreas = separateAreas(areaToCover);

		for (std::vector<DiscreteArea>::iterator isolatedAreaItr = isolatedAreas.begin(); isolatedAreaItr != isolatedAreas.end(); ++isolatedAreaItr)
		{
			// Generate a grid set for the isolated area
			DiscreteArea localCovered(areaToCover.getResolution());
			GridSet solution = gridGenerator.optimalSet(*isolatedAreaItr, localCovered);
	
			// Record the area covered
			areaCovered.insert(localCovered);

			if (solution.size() > 0)
			{
				// Separate the grid into non-connected sets
				std::vector<GridSet> isolatedSets = solution.fourConnectedSets();	
				for (std::vector<GridSet>::iterator itr = isolatedSets.begin(); itr != isolatedSets.end(); ++itr)
				{
					// This check for size is probably unnecessary
					if (itr->size() > 0)
					{
						// Generate the path for the grid set
						STCPath gridPath = pathForGrid(*itr);
						plans.push_back(gridPath);
					}
				}
			}
		}
		return plans;
	}

	std::vector<DiscreteArea> OptimalSTCGenerator::separateAreas(const DiscreteArea& area) const
	{
		// Separate into eight connected sets
		std::list<DiscreteArea> eightConnectedAreas = area.eightConnectedSets< std::list<DiscreteArea> >();

		for (std::list<DiscreteArea>::iterator areaItr = eightConnectedAreas.begin(); areaItr != eightConnectedAreas.end(); ++areaItr)
		{
			// ComparedAreaItr starts one beyond the other iterator
			std::list<DiscreteArea>::iterator comparedAreaItr = areaItr;
			++comparedAreaItr;

			for (; comparedAreaItr != eightConnectedAreas.end(); ++comparedAreaItr)
			{
				// Only combine sets if one of them is small. Two big areas should remain separated
				if (areaItr->size() <= 5 || comparedAreaItr->size() <= 5)
				{
					float distanceBetween = area.getResolution() * minDistanceBetween(*areaItr, *comparedAreaItr);
					if (distanceBetween < 0.15)
					{
						// Combine the two sets and erase the compared one to avoid duplicate
						areaItr->insert(*comparedAreaItr);
						comparedAreaItr = eightConnectedAreas.erase(comparedAreaItr);

						// We may need to break here because comparedAreaItr will be incremented
						// in the for loop before checking != .end()
						if (comparedAreaItr == eightConnectedAreas.end())
							break;
					}
				}
			}	
		}

		return std::vector<DiscreteArea>(eightConnectedAreas.begin(), eightConnectedAreas.end());
	}	

	STCPath OptimalSTCGenerator::pathForGrid(const GridSet& set) const
	{
		STCPath result;

		LowBendTreeBuilder builder;

		// Construct a spanning tree from the grid created earlier
		SpanningTreeGrid spanningTree = builder.buildTree(set); 

		// Create hamiltonian path 
		STCHamiltonian hamiltonianPath;

		// Get the coverage path
		result = hamiltonianPath.generatePath(spanningTree);

		return result;
	}


}

