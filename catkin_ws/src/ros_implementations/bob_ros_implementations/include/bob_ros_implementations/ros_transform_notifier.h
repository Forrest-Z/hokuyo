#ifndef _BOB_ROS_IMPLEMENTATIONS_ROS_TRANSFORM_NOTIFIER_H_
#define _BOB_ROS_IMPLEMENTATIONS_ROS_TRANSFORM_NOTIFIER_H_

#include <bob_sensor/itransform_notifier.h>
#include <bob_tf/transform_broadcaster.h>

namespace bob
{

	class ROSTransformNotifier : public ITransformNotifier
	{

		public:
	
			virtual void updateOdomPose(const Pose2D& odomPose);

			virtual void updateMapToOdom(const Pose2D& pose);

		private:

			Transform toTransform(const Pose2D& pose) const;

			void sendTransform(const Transform& transform, std::string fromFrame, std::string toFrame);

			//! Publishes the transform
			TransformBroadcaster transformBroadcaster;
				

	};

}

#endif
