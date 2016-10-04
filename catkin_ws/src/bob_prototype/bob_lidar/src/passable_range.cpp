#include <bob_lidar/passable_range.h>
#include <bob_config/config.h>
#include <bob_sensor/lidar_scan.h>

namespace bob
{

	AngularRange PassableRange::getPassableRange()
	{
		AngularRange unSafeRange;
		float robotRadius = Config::ROBOT_RADIUS;

		LidarScan rangeData = lidarHandle.getLidarData();
		for(LidarScan::iterator itr = rangeData.begin(); itr != rangeData.end(); ++itr)
		{	
			// Ignore far points and nan range readings
			if(itr->range > robotRadius + maxCheckRange || isnan(itr->range))
				continue;
 
			if(itr->range <= robotRadius + marginDist)
			{
				// If the range reading is smaller than robot radius + desired distance away from obstacle,
				// turn at least to make sure the point is at the side of robot.	
				SimpleAngularRange range(itr->angle - M_PI / 2, itr->angle + M_PI / 2);
				unSafeRange.add(range);			
			}
			else
			{
				// Calculate the angle between desired anglular range boundary and current heading
				float angleToTurnAway = asin((robotRadius + marginDist) / itr->range);
				SimpleAngularRange range(itr->angle - angleToTurnAway, itr->angle + angleToTurnAway);
				unSafeRange.add(range);
			}
		}
		
		// Invert to get the safe range;
		return unSafeRange.invert();
	}



}

