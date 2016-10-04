#include <bob_map_algorithms/obstacle_locator.h>
#include <limits>

#include <bob_map_algorithms/map_functor.h>
#include <bob_config/config.h>
#include <bob_toolbox/grid_algorithms.h>
#include <bob_toolbox/geometry.h>
#include <bob_toolbox/grid_wavefront_full_iterator.h>
#include <bob_toolbox/grid_wavefront_shell_iterator.h>
#include <bob_toolbox/wave_expander.h>
#include <bob_toolbox/set_algorithms.h>
#include <bob_toolbox/wave_expand_get_valid_point.h>

#include <bob_visualization/visualization.h>
#include <bob_visualization/bob_toolbox_visualization.h>
#include <bob_visualization/marker_types.h>

#include <algorithm>

namespace bob
{
	std::vector<MapLocationSet> ObstacleLocator::getGroupedObstacles(const WorldPoint& seed) const 
	{
		MapLocation seedML = map.worldToMap(seed);
		MapLocationSet obstacles = getObstacleSet(seedML);
		return eightConnected<MapLocationSet>(obstacles);
	}

	MapLocationSet ObstacleLocator::getObstacleSet(const MapLocation& seed) const 
	{
		MapLocationSet obstacles;

		MapLocationSet closedSet;

		NotMapFunctor::shared_ptr notFree(new NotMapFunctor(CellStateIs::shared_ptr(new CellStateIs(map, Free))));
		NotMapFunctor::shared_ptr nearObs(new NotMapFunctor(CellAwayFromObstacles::shared_ptr(new CellAwayFromObstacles(map, Config::ROBOT_RADIUS + 0.04))));
		OredMapFunctor::shared_ptr edgeCondition(new OredMapFunctor());
		edgeCondition->add(notFree);
		edgeCondition->add(nearObs);
		
		MapLocation validSeed = seed;

		// Make sure seed doesn't not satisfy the edge condition
		if ((*edgeCondition)(seed))
		{
			//TODO: AND, OR, NOT
			NotMapFunctor::shared_ptr freeAndAwayFromObstacles(new NotMapFunctor(edgeCondition));
			waveExpandGetValidPoint(map, seed, *freeAndAwayFromObstacles, 81, validSeed);
		}


		MapLocationSet obstacleBoundary;
		GridWavefrontShellIterator<MapLocationSet, MapFunctor> waveIterator(closedSet, validSeed, *edgeCondition, obstacleBoundary);		
		waveExpand<MapLocationSet>(waveIterator);
		MapLocationSet edgeCells = waveIterator.getEdge();

		// Remove the points that are not free (erroneous edge points)
		removeFromSetIf(edgeCells, *notFree);

		std::vector<WorldPoint> obstacle_set;
		for(MapLocationSet::const_iterator itr = edgeCells.begin(); itr != edgeCells.end(); ++itr)
		{
			obstacle_set.push_back(map.mapToWorld(*itr));			
		}

		visualizer->visualize("obstacle_area", MarkerSquares(obstacle_set, map.getResolution()), blueMarkerStyle());
		
		return edgeCells;
	}


}

