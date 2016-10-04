#ifndef _BOB_FRONTIER_EXPLORATION_FRONTIER_TRACKER_H_
#define _BOB_FRONTIER_EXPLORATION_FRONTIER_TRACKER_H_

#include <boost/thread.hpp>
#include <bob_sensor/itransform_handle.h>

#include <bob_frontier_exploration/wavefront_frontier_locator.h>

#include <bob_toolbox/internally_locked.h>

#include <bob_grid_map/lockable_map.h>
#include <bob_grid_map/costmap.h>
#include <bob_grid_map/toolbox.h>
#include <bob_toolbox/periodic_thread.h>
#include <bob_system/periodic_threader.h>

namespace bob
{

	class FrontierTrackerThread : public PeriodicThread
	{

		public:

		FrontierTrackerThread(const LockableMap& lockableMap, const ITransformHandle& transformHandle) :
			lockableMap(lockableMap),
			transformHandle(transformHandle),
			initialized(false)
		{}

			//! Algorithm for updating frontier data
			virtual void operator()();

			MapLocationSet getExploredArea() const;

			bool isInitialized() const;

			FrontierCloud getFrontiers() const;

			void reset();

		private:

			void locateFrontiers(const Costmap& costmap);

			MapLocationSet getInitialSet(const Costmap& costmap);

			InternallyLocked<bool> initialized;

			//! Cached most recent frontier data
			InternallyLocked<FrontierCloud> frontierCloud;

			//! Maintain a closed set to avoid checking the same areas twice
			InternallyLocked<MapLocationSet> closedSet;

			void visualize(const FrontierCloud& frontiers, const Costmap& costmap) const;

			const LockableMap& lockableMap;

			const ITransformHandle& transformHandle;

			
	};

	//! Calculates the frontiers on the map, using a separate thread.
	//! The class will periodically calculate the frontiers based on a map.
	class FrontierTracker
	{

		public:

			FrontierTracker(const LockableMap& lockableMap, const ITransformHandle& transformHandle) :
			thread(lockableMap, transformHandle),
			threader(thread, 1.0)
			{}


			inline void start()
			{
				threader.start();
			}

			//! Used to get the most recent frontier information 
			FrontierCloud getFrontiers() const;

			void waitForInitialization() const;

			MapLocationSet getExploredArea() const;	
			
			void reset();

		private:		

			FrontierTrackerThread thread;

			PeriodicThreader threader;

	};

}

#endif
