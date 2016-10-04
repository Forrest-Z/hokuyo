#ifndef _BOB_CONTROL_DISTANCE_CONDITION_H_
#define _BOB_CONTROL_DISTANCE_CONDITION_H_

#include <bob_control/conditions/istop_condition.h>

#include <bob_toolbox/world_point.h>
#include <bob_toolbox/pose2d.h>

namespace bob
{

	//! \brief Condition triggered when the robot moves a certain distance.
	//! Can be configured to use odometry or localized pose. 
	class DistanceCondition : public IStopCondition
	{
		public:

			//! \brief Create a DistanceCondition instance
			DistanceCondition(const ISensorHandle& sensorHandle, float distance, bool useLocalization = false);

		private:

			virtual bool condition(const ISensorHandle& sensorHandle);

			Pose2D desiredRobotPose(const ISensorHandle& sensorHandle);

			bool useLocalization;

			//! Starting point in odom frame;
			WorldPoint startPoint;

			float distance;

			
		
	};

}

#endif
