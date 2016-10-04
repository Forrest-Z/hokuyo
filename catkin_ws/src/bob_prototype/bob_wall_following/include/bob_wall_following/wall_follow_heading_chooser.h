#ifndef _BOB_WALL_FOLLOWING_WALL_FOLLOW_HEADING_CHOOSER_H_
#define _BOB_WALL_FOLLOWING_WALL_FOLLOW_HEADING_CHOOSER_H_

#include <bob_lidar/passable_range.h>
#include <bob_control/wall_follow_side.h>
#include <bob_config/config.h>

namespace bob
{

	class ISensorHandle;

	//! Determines the direction the robot should face in order to align with the wall it is following
	class WallFollowHeadingChooser
	{
		public:

			WallFollowHeadingChooser(const ISensorHandle& sensorHandle, WallFollowSide side):
				sensorHandle(sensorHandle),
				side(side),
				wallFollowDistance(Config::WALL_FOLLOW_DISTANCE)
		{}

			float getHeadingInRobotFrame();

		private:

			const ISensorHandle& sensorHandle;
			WallFollowSide side;
			float wallFollowDistance;

	};

}

#endif
