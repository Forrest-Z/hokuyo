#ifndef _BOB_ROS_IMPLEMENTATIONS_ROS_TRANSFORM_HANDLE_H_
#define _BOB_ROS_IMPLEMENTATIONS_ROS_TRANSFORM_HANDLE_H_

#include <bob_toolbox/pose2d.h>
#include <bob_tf/transform_listener.h>
#include <bob_sensor/itransform_handle.h>

namespace bob
{

	class ROSTransformHandle : public ITransformHandle
	{

		public:

			virtual Pose2D getLocalizedPose() const;

			virtual Pose2D getOdomPose() const;

			virtual void waitForLidarTransform() const;

			virtual void getLidarPose(bool& inverted, float& zeroAngle) const;

		private:

			void waitForTransform(std::string from, std::string to) const;

			Pose2D getFramePose(std::string from, std::string to) const;

			TransformListener transformHandle;

	};

}

#endif
