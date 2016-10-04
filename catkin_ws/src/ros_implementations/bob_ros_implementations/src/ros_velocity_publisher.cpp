#include <bob_ros_implementations/ros_velocity_publisher.h>


namespace bob
{

	void ROSVelocityPublisher::publish(const Velocity2D& command) const
	{
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = command.x;
		cmd_vel.angular.z = command.w;
		velocityPublisher.publish(cmd_vel);
	}

}

