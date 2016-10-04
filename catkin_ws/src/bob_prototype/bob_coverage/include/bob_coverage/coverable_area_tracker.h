#ifndef _BOB_COVERAGE_COVERABLE_AREA_TRACKER_H_
#define _BOB_COVERAGE_COVERABLE_AREA_TRACKER_H_

#include <bob_system/periodic_threader.h>
#include <bob_toolbox/periodic_thread.h>
#include <bob_coverage/discrete_area.h>
#include <bob_toolbox/internally_locked.h>
#include <bob_config/config.h>
#include <bob_frontier_exploration/frontier_tracker.h>
#include <bob_coverage/coverage_tracker.h>

namespace bob
{

	class CoverableAreaTrackerThread : public PeriodicThread
	{

		public:

			CoverableAreaTrackerThread(FrontierTracker& frontierTracker, CoverageTracker& coverageTracker) :
			coverableArea(DiscreteArea(Config::MAP_RESOLUTION)),
			frontierTracker(frontierTracker),
			coverageTracker(coverageTracker)
			{}

			virtual void operator()();

			DiscreteArea getCoverableSpace() const;

		private:		

			InternallyLocked<DiscreteArea> coverableArea;

			FrontierTracker& frontierTracker;
	
			CoverageTracker& coverageTracker;

	};

	class CoverableAreaTracker
	{

		public:

			CoverableAreaTracker(FrontierTracker& frontierTracker, CoverageTracker& coverageTracker, float period) :
				trackerThread(frontierTracker, coverageTracker),
				threader(trackerThread, period)
		{
		}

			inline void start()
			{
				threader.start();
			}
			
			DiscreteArea getCoverableSpace() const;

		private:

			CoverableAreaTrackerThread trackerThread;

			PeriodicThreader threader;	

	};

}

#endif
