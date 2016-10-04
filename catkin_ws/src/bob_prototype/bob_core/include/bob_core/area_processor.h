#ifndef _BOB_CORE_AREA_PROCESSOR_H_
#define _BOB_CORE_AREA_PROCESSOR_H_

#include <bob_grid_map/lockable_map.h>
#include <bob_sensor/itransform_handle.h>
#include <bob_frontier_exploration/frontier_tracker.h>
#include <bob_coverage/coverage_tracker.h>
#include <bob_coverage/discrete_area.h>

namespace bob
{
	//! \brief A class tracks explored area and covered area and processes these area.
	class AreaProcessor 
	{

		public:

			AreaProcessor(const LockableMap& lockableMap, const ITransformHandle& transformHandle) :
				frontierTracker(lockableMap, transformHandle),
				coverageTracker(transformHandle, 0.4)
			{}

			inline void start()
			{
				frontierTracker.start();
				frontierTracker.waitForInitialization();

				coverageTracker.start();	
			}
			
			inline void reset()
			{
				frontierTracker.reset();
			}
						
			inline DiscreteArea getCoverableSpace() const
			{
				DiscreteArea toReturn(0.05);
				toReturn.insert(frontierTracker.getExploredArea());
				toReturn.erase(coverageTracker.getCoveredArea());
				return toReturn;
			}

			inline FrontierCloud getFrontiers() const
			{
				return frontierTracker.getFrontiers();
			}

			inline DiscreteArea getCoveredArea() const
			{
				return coverageTracker.getCoveredArea();
			}

		private:

			FrontierTracker frontierTracker;

			CoverageTracker coverageTracker;

	};

}

#endif
