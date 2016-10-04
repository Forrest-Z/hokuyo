#ifndef _BOB_FRONTIER_EXPLORATION_FRONTIER_CONDITION_H_
#define _BOB_FRONTIER_EXPLORATION_FRONTIER_CONDITION_H_

#include <bob_toolbox/pose2d.h>

#include <bob_frontier_exploration/frontier_tracker.h>

#include <bob_control/conditions/istop_condition.h>

namespace bob
{
	/*

	//! Stop condition for frotiers. Will be true when the check point isn't in frontier cloud.
	class FrontierCondition : public IStopCondition
	{

		public:

			FrontierCondition(FrontierTracker& frontierUpdater, const MapLocation& checkPoint) :
			frontierUpdater(frontierUpdater),
			checkPoint(checkPoint) {}	
	
		private:

			virtual bool condition(const ISensorHandle& sensorHandle);			

			FrontierTracker& frontierUpdater;
			MapLocation checkPoint;
	};
	*/

}

#endif
