#ifndef _BOB_TOOLBOX_WAVE_EXPAND_GET_VALID_POINT_H_
#define _BOB_TOOLBOX_WAVE_EXPAND_GET_VALID_POINT_H_

#include <bob_toolbox/wave_expander.h>
#include <bob_toolbox/grid_wavefront_shell_iterator.h>

#include <bob_grid_map/map_location_set.h>

namespace bob
{
	template <class EdgeConditionFunctor>
	bool waveExpandGetValidPoint(const Costmap& costmap, const MapLocation& seed, const EdgeConditionFunctor& edgeFunctor, const int maxCellToExpand, MapLocation& validPoint)
	{
		//! Wavefront expand
		MapLocationSet closedSet;
		MapLocationSet obstacleBoundary;
		GridWavefrontShellIterator<MapLocationSet, EdgeConditionFunctor> waveIterator(closedSet, seed, edgeFunctor, obstacleBoundary);		
		waveExpand<MapLocationSet>(waveIterator, maxCellToExpand);
		MapLocationSet edgeCells = waveIterator.getEdge();
		if (edgeCells.size() == 0)
		{
			validPoint = seed;
			return false;
		}		
		//! Get the closeset valid point to seed point among candidate points	
		return closestValidPoint(costmap, edgeCells, seed, edgeFunctor, validPoint);
		
	}

	//! Find the closeset valid point to seed point among candidate points	
	template <class EdgeConditionFunctor>
	bool closestValidPoint(const Costmap& costmap, const MapLocationSet& candidatePoints, const MapLocation& seed, const EdgeConditionFunctor& edgeFunctor, MapLocation& validPoint)
	{
		validPoint = seed;
		float minDist = std::numeric_limits<float>::max();
		bool validPointFound = false;

		for (MapLocationSet::const_iterator itr = candidatePoints.begin(); itr != candidatePoints.end(); ++itr)
		{
			if (costmap.getObstacleMap().pointFree(*itr) == Free)
			{
				validPointFound = true;
				float distance = diagonalDistance(*itr, seed);
				if (distance < minDist)
				{
					minDist = distance;
					validPoint = *itr;
				}
			}
		}

		return validPointFound;
	}
}

#endif
