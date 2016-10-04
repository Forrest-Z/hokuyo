#ifndef _BOB_FREERTOS_IMPLEMENTATIONS_FREERTOS_TRANSFORM_HANDLE_H_
#define _BOB_FREERTOS_IMPLEMENTATIONS_FREERTOS_TRANSFORM_HANDLE_H_

#include <bob_sensor/itransform_handle.h>
#include <bob_sensor/itransform_notifier.h>

#include <bob_toolbox/pose2d.h>

namespace bob
{

	class FreeRTOSTransformHandle : public ITransformHandle, public ITransformNotifier
	{

		public:

			virtual void updateMapToOdom(const Pose2D& localizedPoseOffset)
			{
				localizedPose = localizedPoseOffset;
			}

			virtual Pose2D getLocalizedPose() const
			{
				return localizedPose;
			}

			virtual Pose2D getOdomPose() const
			{
				// To implement...
				return Pose2D();
			}

			virtual void waitForLidarTransform() const
			{
				// Do nothing is OK, this function will prbly be removed later
			}

			virtual void getLidarPose(bool& inverted, float& zeroAngle) const
			{
				// This function will also likely be removed
				inverted = true;
				zeroAngle = 0;
			}

		private:

			Pose2D localizedPose;

	};

}

#endif
