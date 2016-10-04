#ifndef _BOB_SENSOR_ITRANSFORM_HANDLE_H_
#define _BOB_SENSOR_ITRANSFORM_HANDLE_H_

namespace bob
{

	class Pose2D;

	//! \brief Provides access to robot transform data. Specifically, provides information
	//! about robot position and orientation.
	class ITransformHandle
	{	

		public:

			//! \brief Obtains the robot pose that is produced using the SLAM system.
			//! \return The pose of the robot, as determined by the SLAM system.
			virtual Pose2D getLocalizedPose() const = 0;

			//! \brief Obtains the raw pose of the robot as produced by the Odometry system.
			//! Individual values for odom pose should not be considered in isolation, and 
			//! they should not be mixed with the localized pose. Instead, the difference
			//! between successive calls to getOdomPose() should be used in algorithms as a
			//! good estimation of how far the robot has moved. The advantage to using the 
			//! odom pose is that it is can change more smoothly than the localized pose.
			//! Also, it is not affected by the SLAM algorithm(s), so you don't need to worry
			//! about any problems or considerations associated with that interdependency.
			//! \return The pose of the robot, as determined by compounded odometry readings
			virtual Pose2D getOdomPose() const = 0;

			virtual void waitForLidarTransform() const = 0;

			//! \brief This function shouldn't really be here. It exists now because we currently
			//! use the tf system to get lidar pose data. This allows us to switch lidar without
			//! having to recompile, but it has the unfortunate consequence that the lidar
			//! configuration depends on the transform data. 
			virtual void getLidarPose(bool& inverted, float& zeroAngle) const = 0;

	};

}

#endif
