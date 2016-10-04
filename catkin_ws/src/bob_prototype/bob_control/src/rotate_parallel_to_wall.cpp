#include <bob_control/rotate_parallel_to_wall.h>

#include <bob_toolbox/logging.h>


namespace bob
{

	void RotateParallelToWall::run()
	{
	
		LidarScan rangeData = lidarHandle.getLidarData();
		
		float shortestValue = std::numeric_limits<float>::max();
                LidarBeam shortestBeam;
		for(LidarScan::iterator it = rangeData.begin(); it != rangeData.end(); ++it)
		{
			// Find the shortest beam and it's index among the wall follow side half
			if(it->range < shortestValue && !isnan(it->range))
			{
				shortestValue = it->range;
				shortestBeam = *it;
			}
		}

		float desiredHeading;
		if (side == RightSide)
			desiredHeading = normalizeAngle(shortestBeam.angle + M_PI/2);
		else
			desiredHeading = normalizeAngle(shortestBeam.angle - M_PI/2);
		LOG_CONTROL("desiredHeading: %f", desiredHeading);
		Pose2D pose = transformHandle.getLocalizedPose();

		commander.execute(HeadingRotationCommand(pose.theta + desiredHeading));		
		
	}


}

