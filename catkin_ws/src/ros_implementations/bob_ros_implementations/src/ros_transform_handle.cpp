#include <bob_ros_implementations/ros_transform_handle.h>

#include <bob_tf/exceptions.h>

#include <bob_toolbox/logging.h>

namespace bob
{

	Pose2D ROSTransformHandle::getLocalizedPose() const
	{
		return getFramePose("map", "base_link");
	}

	Pose2D ROSTransformHandle::getOdomPose() const
	{
		return getFramePose("odom", "base_link");
	}

	void ROSTransformHandle::waitForLidarTransform() const
	{
		waitForTransform("base_footprint", "laser");
	}

	void ROSTransformHandle::waitForTransform(std::string from, std::string to) const
	{
		ros::Duration sleepTime(0.01);
		bool available = false;
		while (!available)
		{
			StampedTransform pose;
			try
			{	
				transformHandle.lookupTransform(from, to, ros::Time(), pose);
				available = true;		
			}
			catch (TransformException ex)
			{}
			sleepTime.sleep();
		}
	}

	Pose2D ROSTransformHandle::getFramePose(std::string from, std::string to) const
	{
		StampedTransform pose;
		try
		{	
			transformHandle.lookupTransform(from, to, ros::Time(0), pose);

		}
		catch (TransformException ex)
		{
			return Pose2D();
		}

		return Pose2D(WorldPoint(pose.getOrigin().getX(), pose.getOrigin().getY()), getYaw(pose.getRotation()));
	}


	void ROSTransformHandle::getLidarPose(bool& inverted, float& zeroAngle) const
	{
		if (!transformHandle.waitForTransform("laser", "base_link", ros::Time(), ros::Duration(5.0)))
		{
			LOG_ERROR("Failed to compute laser pose for lidar processing");
			LOG_ERROR("Check that your transforms are publised correctly.");
			ros::shutdown();
		}	

		// Get the laser's pose, relative to base.
		Stamped<Pose> identity;
		Stamped<Transform> laser_pose;
		identity.setIdentity();
		identity.frame_id_ = "laser";
		try
		{
			transformHandle.transformPose("base_link", identity, laser_pose);
		}
		catch(TransformException e)
		{
			LOG_ERROR("Failed to compute laser pose for lidar processing");
			LOG_ERROR("Check that your transforms are publised correctly.");
			ros::shutdown();
		}

		// Checking whether the laser is pointing up or down by
		// creating a point 1m above the laser position and transform it into the laser - frame
		Vector3 v;
		v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
		Stamped<Vector3> unitVector(v, ros::Time(0), "base_link");
		try
		{
			transformHandle.transformPoint("laser", unitVector, unitVector);
		}
		catch(TransformException& e)
		{
			LOG_ERROR("Failed to compute laser pose for lidar processing");
			LOG_ERROR("Check that your transforms are publised correctly.");
			ros::shutdown();
		}
		inverted = (unitVector.z() < 0);
		zeroAngle = getYaw(laser_pose.getRotation());
	}
}

