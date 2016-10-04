#ifndef _BOB_SENSOR_ITRANSFORM_NOTIFIER_H_
#define _BOB_SENSOR_ITRANSFORM_NOTIFIER_H_ 

namespace bob
{

	class Pose2D;
	//! \brief Updates pose information that is used elsewhwere in the sytem.
	class ITransformNotifier
	{

		public:

			//! \brief Update the localized pose offset
			//! \param transform The transform from map to odom frame
			virtual void updateMapToOdom(const Pose2D& localizedPoseOffset) = 0;

	};

}

#endif
