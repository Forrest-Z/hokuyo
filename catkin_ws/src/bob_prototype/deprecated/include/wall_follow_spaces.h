#ifndef _BOB_CONTROL_WALL_FOLLOW_SPACES_H_
#define _BOB_CONTROL_WALL_FOLLOW_SPACES_H_

#include <bob_control/icontroller.h>

#include <bob_toolbox/velocity2d.h>
#include <bob_toolbox/pose2d.h>

#include <bob_toolbox/map_half_space.h>

#include <bob_toolbox/circular_direction.h>
#include <bob_lidar/lidar_processor.h>

#include <ros/ros.h>

#include <bob_tf/transform_listener.h>

namespace bob
{

	class WallFollowSpaces
	{

		LidarWallFollower& lidarWallFollower;		

		float freeAngle;

		MapHalfSpace endEarly;
		MapHalfSpace goalSpace;
		MapHalfSpace failSpace;

		ros::Subscriber laserSubscriber;

		sensor_msgs::LaserScan msgBuffer;

		LidarProcessor lidarProcessor;

		void laserMessageCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

		bool canMoveForward(float robotHeading);

		public:

		WallFollowSpaces(LidarWallFollower& base, float freeAngle, MapHalfSpace endEarly, MapHalfSpace goalSpace, MapHalfSpace failSpace, LidarWallFollower::WallFollowSide side, TransformListener& transformHandle) :
			freeAngle(freeAngle),
			lidarWallFollower(base),
			endEarly(endEarly),
			goalSpace(goalSpace),
			failSpace(failSpace),
			lidarProcessor(transformHandle)
		{
			lidarWallFollower.init(0, side, 0.05);

			ros::NodeHandle nh;
			laserSubscriber = nh.subscribe("scan", 1, &WallFollowSpaces::laserMessageCallback, this);
		}


		enum ControllerState
		{
			Continuing,
			ReturnedToPath,
			GoalReached,
			Failed
		};

		Velocity2D nextCommand(const ControlState& robotState, ControllerState& state);

	};

}

#endif
