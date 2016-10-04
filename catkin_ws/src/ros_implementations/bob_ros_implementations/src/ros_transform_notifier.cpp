#include <bob_ros_implementations/ros_transform_notifier.h>
#include <bob_toolbox/pose2d.h>
#include <bob_toolbox/logging.h>


namespace bob
{

	void ROSTransformNotifier::updateOdomPose(const Pose2D& odomPose)
	{
		sendTransform(toTransform(odomPose), "odom", "base_link");
	}

	void ROSTransformNotifier::updateMapToOdom(const Pose2D& pose)
	{
		sendTransform(toTransform(pose), "map", "odom");
	}

	void ROSTransformNotifier::sendTransform(const Transform& transform, std::string fromFrame, std::string toFrame)
	{
		transformBroadcaster.sendTransform(StampedTransform(transform, ros::Time::now(), fromFrame, toFrame));
	}

	Transform ROSTransformNotifier::toTransform(const Pose2D& pose) const
	{
		return Transform(createQuaternionFromRPY(0, 0, pose.theta), Vector3(pose.x, pose.y, 0.0));
	}
}

