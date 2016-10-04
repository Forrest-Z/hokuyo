#include <bob_ros_implementations/ros_odometry_handle.h>

namespace bob 
{

	ROSOdometryHandle::ROSOdometryHandle() 
	{
		ros::NodeHandle gn;
		odomSubscriber = gn.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind( &ROSOdometryHandle::odomCallback, this, _1 ));
	}

	void ROSOdometryHandle::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
	{
		// Storing odom data from callback
		odomData.twist.twist.linear.x = msg->twist.twist.linear.x;
		odomData.twist.twist.linear.y = msg->twist.twist.linear.y;
		odomData.twist.twist.angular.z = msg->twist.twist.angular.z;
		odomData.child_frame_id = msg->child_frame_id;
	}

	Velocity2D ROSOdometryHandle::getRobotVel() const
	{
		Velocity2D velocity;

		velocity.x = odomData.twist.twist.linear.x;
		velocity.w = odomData.twist.twist.angular.z;
		return velocity;
	}
}
