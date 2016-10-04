#ifndef _BOB_FRONTIER_EXPLORATION_FRONTIER_EXPLORER_H_
#define _BOB_FRONTIER_EXPLORATION_FRONTIER_EXPLORER_H_

#include <bob_frontier_exploration/frontier_tracker.h>
#include <bob_frontier_exploration/frontier_chooser.h>
#include <bob_frontier_exploration/frontier_condition.h>
#include <bob_control/straight_driver.h>
#include <bob_navigation/navigation_manager.h>


namespace bob
{

	//! Explores all the existing frontiers between known area and unknown area.
	class FrontierExplorer
	{

		public:

			FrontierExplorer(const LockableMap& lockableMap, const ITransformHandle& transformHandle, SimpleCommander& simpleCommander, CurvedPathExecutor& pathExecutor, OdometryHelper& odometryHelper):
				lockableMap(lockableMap),
				transformHandle(transformHandle),
				simpleCommander(simpleCommander),
				frontierTracker(lockableMap, transformHandle),
				frontierChooser(lockableMap, frontierTracker, transformHandle),
				navigationManager(pathExecutor, lockableMap, transformHandle, odometryHelper)
		{}

			void exploreFrontiers();


		private:

			const LockableMap& lockableMap;
			const ITransformHandle& transformHandle;
			SimpleCommander& simpleCommander;
			FrontierTracker frontierTracker;			
			FrontierChooser frontierChooser;
			NavigationManager navigationManager;		

	};

}

#endif
