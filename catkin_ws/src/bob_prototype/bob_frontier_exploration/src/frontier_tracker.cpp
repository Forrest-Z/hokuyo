#include <bob_frontier_exploration/frontier_tracker.h>

#include <bob_map_algorithms/map_functor.h>
#include <bob_visualization/visualization.h>

#include <bob_navigation/path_splitter.h>
#include <bob_visualization/bob_toolbox_visualization.h>

#include <bob_config/config.h>

#include <bob_toolbox/pose2d.h>
#include <bob_toolbox/strict_lock.h>
#include <bob_toolbox/wave_expand_get_valid_point.h>
#include <bob_system/system_utilities.h>

namespace bob
{
	void FrontierTracker::waitForInitialization() const
	{
		while (!thread.isInitialized())
		{
			systemUtilities->sleep(0.1);
		}
	}

	void FrontierTracker::reset() 
	{
		thread.reset();
	}

	FrontierCloud FrontierTracker::getFrontiers() const
	{
		return thread.getFrontiers();
	}
	
	MapLocationSet FrontierTracker::getExploredArea() const
	{
		return thread.getExploredArea();
	}

	void FrontierTrackerThread::operator()()
	{
		// Lock map
		StrictLock mapLock(lockableMap.getLock());
		const Costmap& costmap = lockableMap.getLockedResource(mapLock);	

		// The actual frontier location algorithm
		locateFrontiers(costmap);

		// Visualize result
		visualize(frontierCloud.getValue(), costmap);	
	}

	void FrontierTrackerThread::reset()
	{
		// Reset closeSet
		MapLocationSet emptyCloseSet;
		closedSet.setValue(emptyCloseSet);

		// Reset frontiers
		FrontierCloud emptyFrontier;
		frontierCloud.setValue(emptyFrontier);
		
		initialized.setValue(false);

		// Reinitialize frontiers
		// Lock map
		StrictLock mapLock(lockableMap.getLock());
		const Costmap& costmap = lockableMap.getLockedResource(mapLock);	

		// The actual frontier location algorithm
		locateFrontiers(costmap);
	}

	void FrontierTrackerThread::locateFrontiers(const Costmap& costmap)
	{
		// Algorithm that actually calculates frontiers
		MapLocationSet seedSet;
		FrontierCloud previousFrontiers = frontierCloud.getValue();
		if (previousFrontiers.size() == 0)
		{
			seedSet = getInitialSet(costmap);
		}
		else
		{
			// If we have previous frontiers, then use them as seed and also used past closed set
			// This is more efficient: we don't explore backwards where we already know there is not a frontier
			seedSet = previousFrontiers;
		}

		WavefrontFrontierLocator wavefrontFrontierLocator(costmap);
		MapLocationSet tempClosedSet = closedSet.getValue();
		FrontierCloud newCloud = wavefrontFrontierLocator.getFrontiers(tempClosedSet, seedSet);
		
		// Thread safety
		closedSet.setValue(tempClosedSet);
		frontierCloud.setValue(newCloud); 
		initialized.setValue(true);
	}

	MapLocationSet FrontierTrackerThread::getInitialSet(const Costmap& costmap)
	{
		// If there are no frontiers then we use an empty closed set and seed with the robot's current location
		MapLocation robotMapLocation = costmap.worldToMap(transformHandle.getLocalizedPose());
		robotMapLocation.x += 1; // hokuyo limitation
		
		// robot current location has to be free and away from obstacles, if not, expand to get a clear point
		CellStateIs::shared_ptr free(new CellStateIs(costmap, Free));
		CellAwayFromObstacles::shared_ptr awayFromObs(new CellAwayFromObstacles(costmap, Config::ROBOT_RADIUS + 0.04));
		AndedMapFunctor::shared_ptr edgeCondition(new AndedMapFunctor());
		edgeCondition->add(free);
		edgeCondition->add(awayFromObs);

		if (!(*edgeCondition)(robotMapLocation))
		{
			// robot is close to obstacle
			MapLocation validSeed;
			waveExpandGetValidPoint(costmap, robotMapLocation, *edgeCondition, 169, validSeed);
			robotMapLocation = validSeed;
		}
		
		MapLocationSet toReturn;
		toReturn.insert(robotMapLocation);
		return toReturn;
	}

	FrontierCloud FrontierTrackerThread::getFrontiers() const
	{
		return frontierCloud.getValue();
	}

	void FrontierTrackerThread::visualize(const FrontierCloud& frontiers, const Costmap& costmap) const
	{
		std::vector<WorldPoint> frontierPoints = costmap.mapToWorld<std::vector<WorldPoint> >(frontiers);
		visualizer->visualize("frontiers", MarkerSquares(frontierPoints, costmap.getResolution()));
	}

	MapLocationSet FrontierTrackerThread::getExploredArea() const
	{
		//DiscreteArea toReturn(lockableMap.getLocationMapper.getResolution(), closed.begin(), closed.end());
		return closedSet.getValue();
	}

	bool FrontierTrackerThread::isInitialized() const
	{
		return initialized.getValue();
	}
	
}

