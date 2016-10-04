#ifndef _BOB_CONTROL_WALL_FOLLOW_SEGMENT_CONTROLLER_H_
#define _BOB_CONTROL_WALL_FOLLOW_SEGMENT_CONTROLLER_H_

#include <bob_control/icontroller.h>
#include <bob_toolbox/pose2d.h>
#include <bob_toolbox/map_half_space.h>

namespace bob
{

	class WallFollowSegmentController : public IController
	{
		private:
			MapHalfSpace goalRegion;			
			LidarWallFollower& lidarWallFollower;
		public:
			WallFollowSegmentController(WorldPoint lineOrigin, float normalAngle, CircularDirection direction, float wallFollowDistance, LidarWallFollower& lidarWallFollower);

			virtual Velocity2D nextCommand(Pose2D robotPose, Velocity2D robotVelocity, bool& commandFound, bool& goalReached);
	};

}

#endif
