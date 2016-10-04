#include <bob_wall_following/wall_follow_spaces.h>

#include <bob_lidar/util_functions.h>

namespace bob
{

	Velocity2D WallFollowSpaces::nextCommand(const ControlState& robotState, ControllerState& state)
	{
		if (endEarly.contains(robotState.pose) && canMoveForward(robotState.pose.theta))
		{
			state = ReturnedToPath;
		}
		else if (failSpace.contains(robotState.pose))
		{
			state = Failed;
		}
		else if (goalSpace.contains(robotState.pose))
		{
			state = GoalReached;
		}
		else
		{
			state = Continuing;
		}

		return lidarWallFollower.nextCommand(robotVelocity);
	}

	bool WallFollowSpaces::canMoveForward(float robotHeading)
	{
		// We use (freeAngle - robotHeading) because distanceFromRobot is in the frame of the robot, not the world!
		float distanceAvailable = distanceFromRobot(msgBuffer, lidarProcessor, 0.177, freeAngle - robotHeading);			
		return (distanceAvailable > 0.2);
	}

	void WallFollowSpaces::laserMessageCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
	{
		msgBuffer = *msg;
	}
}

