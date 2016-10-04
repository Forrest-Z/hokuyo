#include <bob_wall_following/wall_follow_segment_controller.h>

namespace bob
{

	WallFollowSegmentController::WallFollowSegmentController(WorldPoint lineOrigin, float normalAngle, CircularDirection direction, float wallFollowDistance, LidarWallFollower& lidarWallFollower):
	goalRegion(lineOrigin, normalAngle),
	lidarWallFollower(lidarWallFollower)
	{
		LidarWallFollower::WallFollowSide side = (direction == Clockwise) ? LidarWallFollower::Right : LidarWallFollower::Left;
		lidarWallFollower.init(normalAngle, side, wallFollowDistance);
	}

	Velocity2D WallFollowSegmentController::nextCommand(Pose2D robotPose, Velocity2D robotVelocity, bool& commandFound, bool& goalReached)
	{
		commandFound = true;
		
		if(goalRegion.contains(robotPose))
		{
			goalReached = true;
		}
		
		return lidarWallFollower.nextCommand(robotVelocity);
	}

}

