#ifndef _BOB_CORE_ALTERNATING_EXPLORATION_COVERAGE_H_
#define _BOB_CORE_ALTERNATING_EXPLORATION_COVERAGE_H_

#include <bob_control/conditions/istop_condition.h>

#include <bob_coverage/coverage_tracker.h>
#include <bob_coverage/coverable_area_tracker.h>
#include <bob_coverage/icoverage_algorithm.h>
#include <bob_frontier_exploration/frontier_tracker.h>

#include <bob_sensor/isensor_handle.h>
#include <bob_frontier_exploration/frontier_chooser.h>
#include <bob_control_handle/control_handle.h>
#include <bob_core/area_processor.h>

namespace bob
{
	class ExploredEnoughCondition : public IStopCondition
	{
		public:

			ExploredEnoughCondition(const AreaProcessor& areaProcessor) : 
				areaProcessor(areaProcessor)
		{}

			virtual bool condition(const ISensorHandle& sensorHandle);

		private:

			const AreaProcessor& areaProcessor;
	};

	//! Stop condition for frotiers. Will be true when the check point isn't in frontier cloud.
	class FrontierCondition : public IStopCondition
	{

		public:

			FrontierCondition(AreaProcessor& areaProcessor, const MapLocation& checkPoint) :
			areaProcessor(areaProcessor),
			checkPoint(checkPoint) {}	
	
		private:

			virtual bool condition(const ISensorHandle& sensorHandle);			

			AreaProcessor& areaProcessor;
			MapLocation checkPoint;
	};

	class AlternatingExplorationCoverage
	{
		private:

			enum ExplorationResult
			{
				SwitchToCoverage,
				CompletelyExplored
			};

		public:

			AlternatingExplorationCoverage(const LockableMap& lockableMap, const ISensorHandle& sensorHandle, ControlHandle& controlHandle, AreaProcessor& areaProcessor, ICoverageAlgorithm& coverageAlgorithm) :
			lockableMap(lockableMap),
			sensorHandle(sensorHandle),
			controlHandle(controlHandle),
			coverageAlgorithm(coverageAlgorithm),
			areaProcessor(areaProcessor),
			frontierChooser(lockableMap, sensorHandle.getTransformHandle())
			{}
	
			void operator()();

			ExplorationResult exploration();

			void coverage();

		private:

			void removeSmallAreas(std::vector<DiscreteArea>& areas, size_t minSize);

			void removeAreaNearObstacle(float obstacleDistance, DiscreteArea& area);

			const LockableMap& lockableMap;
			const ISensorHandle& sensorHandle;
			ControlHandle& controlHandle;
		
			ICoverageAlgorithm& coverageAlgorithm;

			AreaProcessor& areaProcessor;

			FrontierChooser frontierChooser;			

			//! The points which have already been visited
			MapLocationSet triedPoints;

	};

}

#endif
