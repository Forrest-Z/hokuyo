#include <bob_frontier_exploration/wavefront_frontier_locator.h>

#include <bob_frontier_exploration/frontier_cloud.h>

#include <vector>
#include <bob_visualization/bob_toolbox_visualization.h>

#include <bob_map_algorithms/map_functor.h>

#include <bob_toolbox/grid_wavefront_full_iterator.h>
#include <bob_toolbox/grid_wavefront_shell_iterator.h>

#include <bob_toolbox/wave_expander.h>
#include <bob_toolbox/set_algorithms.h>
#include <bob_visualization/visualization.h>

#include <bob_grid_map/costmap.h>

#include <bob_config/config.h>

#include <assert.h>

namespace bob
{

	FrontierCloud WavefrontFrontierLocator::getFrontiers(MapLocationSet& closedSet, MapLocation seed) const
	{
		MapLocationSet set;
		set.insert(seed);
		return getFrontiers(closedSet, set);
	}

	FrontierCloud WavefrontFrontierLocator::getFrontiers(MapLocationSet& closedSet, MapLocationSet seed) const
	{
		//TODO: Check seed valid
		NotMapFunctor::shared_ptr notFree(new NotMapFunctor(CellStateIs::shared_ptr(new CellStateIs(map, Free))));
		NotMapFunctor::shared_ptr nearObs(new NotMapFunctor(CellAwayFromObstacles::shared_ptr(new CellAwayFromObstacles(map, Config::ROBOT_RADIUS + 0.04))));
		OredMapFunctor::shared_ptr edgeCondition(new OredMapFunctor());
		edgeCondition->add(notFree);
		edgeCondition->add(nearObs);
		
		/*
		NotMapFunctor::shared_ptr notEdge(new NotMapFunctor(edgeCondition));
		for(MapLocationSet::iterator itr = seed.begin(); itr != seed.end(); ++itr)
		{
			LOG("(%d, %d) not edge: %d", itr->x, itr->y, (*notEdge)(*itr));
		}
		*/

		GridWavefrontFullIterator<MapLocationSet, MapFunctor> waveIterator(closedSet, seed, *edgeCondition);		
		waveExpand<MapLocationSet>(waveIterator);

		MapLocationSet edgeCells = waveIterator.getEdge();
		std::vector<WorldPoint> shell;
		for(MapLocationSet::const_iterator itr = edgeCells.begin(); itr != edgeCells.end(); ++itr)
		{
			shell.push_back(map.mapToWorld(*itr));
		}
		visualizer->visualize("edge", MarkerSquares(shell, map.getResolution()));

		// Remove the points that are not unknown (erroneous edge points)
		CellStateIs::shared_ptr unknown(new CellStateIs(map, Unknown));
		NotMapFunctor::shared_ptr notUnknown(new NotMapFunctor(unknown));
		
		removeFromSetIf(edgeCells, *notUnknown);

		return edgeCells;
	}
}
