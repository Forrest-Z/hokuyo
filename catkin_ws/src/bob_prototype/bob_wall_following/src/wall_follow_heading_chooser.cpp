#include <bob_wall_following/wall_follow_heading_chooser.h>

#include <bob_sensor/isensor_handle.h>
#include <bob_sensor/iproximity_sensor.h>
#include <bob_toolbox/logging.h>

namespace bob
{
	float WallFollowHeadingChooser::getHeadingInRobotFrame()
	{
		AngularRange range = sensorHandle.getProximitySensor().getPassableRange(0.15, wallFollowDistance);
		AngularRange unpassableRange = range.invert();
		std::list<SimpleAngularRange> ranges = range.getRanges();
		
		if(ranges.empty())
		{
			// No feasible direction
			LOG_ERROR("Robot stuck in a closed area!");
		}		
		
		if(ranges.front().isFullCircle())
		{
			// Every direction works, keep current heading
			return 0;
		}

		if (range.isInside(0))
		{
			SimpleAngularRange thatIsInside = range.containingSubrange(0);	
			if(side == RightSide)
			{
				// If doing right side wall following, check counter clockwise start from current heading
				// find the first feasible angle; 
				return thatIsInside.lower;
			}
			else
			{
				// If doing left side wall following, check clockwise start from current heading,
				// find the first feasible angle; 
				return thatIsInside.upper;
			}
		}
		else
		{
			SimpleAngularRange thatIsInside = unpassableRange.containingSubrange(0);	
			if(side == LeftSide)
			{
				// If doing right side wall following, check counter clockwise start from current heading
				// find the first feasible angle; 
				return thatIsInside.lower;
			}
			else
			{
				// If doing left side wall following, check clockwise start from current heading,
				// find the first feasible angle; 
				return thatIsInside.upper;
			}
		}


	}

}

