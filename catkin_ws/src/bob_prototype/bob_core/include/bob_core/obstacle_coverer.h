#ifndef _BOB_CORE_OBSTACLE_COVERER_H_
#define _BOB_CORE_OBSTACLE_COVERER_H_

#include <bob_coverage/coverage_tracker.h>

namespace bob
{

	class ObstacleCoverer
	{
		public:
			
			ObstacleCoverer(MapLocationSet obstacle, ITransformHandle& transformHandle) : 
				obstacle(obstacle),
				coverageTracker(transformHandle)
			{}
			
			void execute();
			
		private:

			MapLocationSet obstacle;
			
			CoverageTracker coverageTracker;
			
	};

}

#endif
