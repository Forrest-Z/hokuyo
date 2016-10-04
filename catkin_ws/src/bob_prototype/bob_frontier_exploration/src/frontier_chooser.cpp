#include <bob_frontier_exploration/frontier_chooser.h>

#include <bob_toolbox/geometry.h>
#include <bob_toolbox/angles.h>
#include <bob_toolbox/easy_print.h>
#include <bob_toolbox/pose2d.h>
#include <boost/bind.hpp>
#include <algorithm>

namespace bob
{

	FrontierChooser::MapWorldPair FrontierChooser::next(FrontierCloud frontierCloud)
	{
		Pose2D currentPose = transformHandle.getLocalizedPose();
		MapWorldPair result = chooseFrontier(frontierCloud, currentPose);
		return result;
	}

	FrontierChooser::MapWorldPair FrontierChooser::chooseFrontier(const FrontierCloud& cloud, Pose2D currentRobotPose)
	{
		std::list<Sortable> dataForSorting = createSortableVector(cloud, currentRobotPose);
		std::list<Sortable>::iterator frontierItr = std::min_element(dataForSorting.begin(), dataForSorting.end(), boost::bind(&FrontierChooser::isBetterThan, *this, _1, _2));
		return frontierItr->pair;
	}

	std::list<FrontierChooser::Sortable> FrontierChooser::createSortableVector(const FrontierCloud& cloud, Pose2D currentRobotPose) const
	{
		StrictLock lock(lockableMap.getLock());
		const Costmap& costmap = lockableMap.getLockedResource(lock);

		std::list<Sortable> result;
		for (FrontierCloud::const_iterator itr = cloud.begin(); itr != cloud.end(); ++itr)
		{
			WorldPoint point = costmap.mapToWorld(*itr);

			Sortable newEntry;
			newEntry.pair.location = *itr;
			newEntry.pair.point = point;
			newEntry.distanceFromObstacles = costmap.getObstacleDistanceMap().obstacleDistance(*itr);
			newEntry.distanceFromRobot = diagonalDistance<WorldPoint>(currentRobotPose, point);
			newEntry.headingFromRobot = normalizeAngle(rayAngle<WorldPoint>(currentRobotPose, point) - currentRobotPose.theta);
			result.push_back(newEntry);
		}
		return result;
	}

	bool FrontierChooser::isBetterThan(const Sortable& first, const Sortable& second) const
	{
		float headingComparison = fabs(second.headingFromRobot) - fabs(first.headingFromRobot);
		//return first.distanceFromObstacles > second.distanceFromObstacles;
		if (headingComparison > 1.57)
		{
			// The second is much further outside of the robot heading (90 degrees difference)
			return true;
		}
		else
		{
			return first.distanceFromObstacles > second.distanceFromObstacles;
		}
	}

}

