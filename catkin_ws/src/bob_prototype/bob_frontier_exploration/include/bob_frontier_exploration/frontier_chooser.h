#ifndef _BOB_FRONTIER_EXPLORATION_FRONTIER_CHOOSER_H_
#define _BOB_FRONTIER_EXPLORATION_FRONTIER_CHOOSER_H_

#include <bob_frontier_exploration/frontier_tracker.h>
#include <bob_grid_map/lockable_map.h>
#include <bob_toolbox/pose2d.h>

namespace bob
{

	//! This class is used to decide which of the frontiers to explore next.

	//! The previously explored frontiers are stored in a set, so that they are not retried.
	class FrontierChooser
	{

		public:

			FrontierChooser(const LockableMap& lockableMap, const ITransformHandle& transformHandle): 
				lockableMap(lockableMap),
				transformHandle(transformHandle) {}

			struct MapWorldPair
			{
				MapLocation location;
				WorldPoint point;
			};

			//! Given a cloud of frontiers, which one should be explored next?
			MapWorldPair next(FrontierCloud frontierCloud);

		private:

			const LockableMap& lockableMap;
			const ITransformHandle& transformHandle;

			struct Sortable
			{
				MapWorldPair pair;
				float distanceFromRobot;
				float headingFromRobot;
				float distanceFromObstacles;
			};

			MapWorldPair chooseFrontier(const FrontierCloud& cloud, Pose2D currentRobotPose);

			bool isBetterThan(const Sortable& first, const Sortable& second) const;

			std::list<Sortable> createSortableVector(const FrontierCloud& cloud, Pose2D currentRobotPose) const;

	};

}

#endif
