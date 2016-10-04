#include <bob_core/alternating_exploration_coverage.h>
#include <bob_core/area_coverer.h>
#include <bob_coverage/area_tools.h>
#include <bob_coverage/visualization.h>
#include <bob_frontier_exploration/frontier_condition.h>
#include <bob_control/conditions/ored_condition.h>

#include <bob_system/periodic_threader.h>
#include <bob_visualization/bob_toolbox_visualization.h>
#include <bob_toolbox/set_algorithms.h>
#include <bob_toolbox/logging.h>
#include <bob_visualization/visualization.h>
#include <bob_map_algorithms/map_functor.h>
#include <bob_system/system_utilities.h>

#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <cmath>

namespace bob
{
	void AlternatingExplorationCoverage::operator()()
	{
		while (systemUtilities->ok())
		{
			LOG_HIGH_LEVEL("Switchin to frontier exploration");
			ExplorationResult result = exploration();

			LOG_HIGH_LEVEL("Switching to coverage");
			coverage();

			// If the area was completely explored in the last exploration cycle, then return
			if (result == CompletelyExplored)
			{
				// Visualize the area after coverage.
				visualizer->visualize("total_area", visualization(areaProcessor.getCoverableSpace()));
				break;
			}
		}
	}

	bool ExploredEnoughCondition::condition(const ISensorHandle& sensorHandle)
	{
		// Calculate the floor space that is available to be covered
		int coverableSize = areaProcessor.getCoverableSpace().size();
		float coverableMetersSquared = coverableSize * pow(Config::MAP_RESOLUTION, 2);	

		// Switch to coverage if a certain amount is available
		if (coverableMetersSquared > 2)
			return true;
		else
			return false;
	}

	bool FrontierCondition::condition(const ISensorHandle& sensorHandle)
	{
		bool frontierExist;
		FrontierCloud frontierCloud = areaProcessor.getFrontiers();
		return ((frontierCloud.size() == 0) || frontierCloud.count(checkPoint) == 0);
	}			

	AlternatingExplorationCoverage::ExplorationResult AlternatingExplorationCoverage::exploration()
	{
		ExploredEnoughCondition::shared_ptr exploredEnoughCondition(new ExploredEnoughCondition(areaProcessor));

		// Explore until no more frontiers or we have explored enough space
		while(true)
		{
			FrontierCloud newFrontiers = areaProcessor.getFrontiers();

			for (MapLocationSet::iterator triedItr = triedPoints.begin(); triedItr != triedPoints.end(); ++triedItr)
			{
				newFrontiers.erase(*triedItr);
			}

			// Entire map now explored, no new frontiers remain
			if (newFrontiers.size() == 0)
			{
				return CompletelyExplored;	
			}

			FrontierChooser::MapWorldPair exploreGoal = frontierChooser.next(newFrontiers);
			triedPoints.insert(exploreGoal.location);

			OredCondition::shared_ptr condition(new OredCondition);

			// Condition triggered when frontier disappears
			FrontierCondition::shared_ptr frontierCondition(new FrontierCondition(areaProcessor, exploreGoal.location));		
			condition->add(frontierCondition);

			// Condition triggered when enough area has been explored to switch to coverage
			condition->add(exploredEnoughCondition);

			// Navigate to the frontier
			auto result = controlHandle.navigationManager.navigationGoal(exploreGoal.point, condition);	
		
			if (exploredEnoughCondition->wasSatisfied())
				return SwitchToCoverage;
		}
		// Slow down		
		/*
		   Pose2D currPose = sensorHandle.getTransformHandle().getLocalizedPose();
		   AccelerateCommand slowdown(Line(currPose, currPose.theta), 0.05, 0.0);
		   controlHandle.simpleCommander.execute(slowdown);
		 */
	}

	void AlternatingExplorationCoverage::coverage()
	{
		std::vector<DiscreteArea> areas;
		do
		{
			// Update coverable area
			DiscreteArea area = areaProcessor.getCoverableSpace();
			size_t coveredAreaSizeBefore = areaProcessor.getCoveredArea().size();
			visualizer->visualize("total_area", visualization(area));
			
			// Remove areas near obstacles
			removeAreaNearObstacle(0.4, area);

			if (area.size() == 0)
				break;

			// Group areas into connected sets
			areas = area.eightConnectedSets<std::vector<DiscreteArea> >();
			
			// Remove areas smaller than 10
			removeSmallAreas(areas, 10);

			if (areas.size() == 0)
				break;

			// Get the closest area
			WorldPoint robotPosition = sensorHandle.getTransformHandle().getLocalizedPose();
			auto itrClosestArea = closestArea(areas, robotPosition);

			// Cover the closest area
			visualizer->visualize("area_portion", visualization(*itrClosestArea));
			coverageAlgorithm.coverArea(*itrClosestArea);

			size_t coveredAreaSizeAfter = areaProcessor.getCoveredArea().size();

			LOG_HIGH_LEVEL("area difference: " << (coveredAreaSizeAfter - coveredAreaSizeBefore));
			if (coveredAreaSizeAfter - coveredAreaSizeBefore < 5)
			{
				areaProcessor.reset();
			}

		}		
		while(areas.size() > 0);
		
	}

	void AlternatingExplorationCoverage::removeSmallAreas(std::vector<DiscreteArea>& areas, size_t minSize)
	{
		for (auto itr = areas.begin(); itr != areas.end();)	
		{
			if (itr->size() < minSize)
			{
				itr = areas.erase(itr);
			}
			else
			{
				++itr;
			}
		}
	}

		
	void AlternatingExplorationCoverage::removeAreaNearObstacle(float obstacleDistance, DiscreteArea& area)
	{
		// Remove areas close to obstacles
		StrictLock lock(lockableMap.getLock());
		const Costmap& costmap = lockableMap.getLockedResource(lock);
		CellAwayFromObstacles::shared_ptr awayFromObs(new CellAwayFromObstacles(costmap, obstacleDistance));
		NotMapFunctor::shared_ptr nearObs(new NotMapFunctor(awayFromObs));

		removeFromSetIf(area.getRawData(), *nearObs);
	}
}

