#include <bob_wall_following/wall_follow_overlap_condition.h>
#include <bob_toolbox/geometry.h>
#include <bob_visualization/visualization.h>
#include <bob_visualization/bob_toolbox_visualization.h>
#include <bob_toolbox/angles.h>
#include <bob_sensor/itransform_handle.h>

namespace bob
{

	bool WallFollowOverlapCondition::condition(const ISensorHandle& sensorHandle)
	{
		// Get robot Pose
		Pose2D robotPose = sensorHandle.getTransformHandle().getLocalizedPose();
		 
		if (!initialized)
		{
			// Initialization
			nextCheckPoint = robotPose;
			lastAddTime->startTimer();
			initialized = true;
		}
	
		// Limit number of check points to avoid high cost calculation and memory usage	
		if ( checkList.size() <= maxNumOfCheckPoint)
		{
			// If the lastAddTime is more than 10 seconds ago and the current nextCheckPoint
			// has been added, update nextCheckPoint.
			if (lastAddTime->timeElapsed() > period && nextAdded)
			{
				nextCheckPoint = robotPose;
				nextAdded = false;
				period = checkList.size() <= 10 ? 10 : period + 1;
			}		

			// If the current nextCheckPoint has not been added to the checkList, 
			// update the checkList
			if (!nextAdded)
			{
				updateCheckList(robotPose);		
			}
		}
		
		// Match robotPose with every pose stored in checkList from the oldest to the newest,
		// return true when matches.
		return check(robotPose);
	}


	void WallFollowOverlapCondition::updateCheckList(const Pose2D& robotPose)
	{
		// The distance from current robot position to the nextCheckPoint stored.
		float distanceToNext = diagonalDistance<WorldPoint>(robotPose, nextCheckPoint);

		// When the robot is 0.2 away from the nextCheckPoint, update the lastAddTime 
		// and turn on nextAdded flag.
		if (distanceToNext > 0.2)// TODO: add to config
		{
			checkList.push_back(nextCheckPoint);
			lastAddTime->startTimer();
			nextAdded = true;	

			// Visualization
			//visualizer->visualize("visited_point", visualization(checkList), purpleMarkerStyle());
		}
	}
	
	bool WallFollowOverlapCondition::check(const Pose2D& robotPose)
	{
		if (!checkList.empty())
		{
			// Match robotPose with every pose stored in checkList from the oldest to the newest,
			// return true when matches.
			for(std::vector<Pose2D>::const_iterator itr = checkList.begin(); itr != checkList.end(); ++itr)
			{
				//TODO: refactor and config
				if (diagonalDistance<WorldPoint>(robotPose, *itr) < 0.15)
				{
				 	double angleDiff = fabs(normalizeAngle(robotPose.theta - itr->theta));
					if (angleDiff < M_PI / 3)
					{
						return true;
					}
				}
			}		
		}
		
		return false;
	}	

}


