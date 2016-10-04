#ifndef _BOB_ROS_IMPLEMENTATIONS_ROS_VELOCITY_PUBLISHER_H_
#define _BOB_ROS_IMPLEMENTATIONS_ROS_VELOCITY_PUBLISHER_H_

#include <ros/ros.h>
#include <bob_toolbox/velocity2d.h>
#include <bob_control/ivelocity_publisher.h>
#include <geometry_msgs/Twist.h>

namespace bob
{

	//! \brief A ROS implementation of IVelocityPublisher, which sends velocity commands to the kobuki base
	class ROSVelocityPublisher : public IVelocityPublisher
	{

		public:

			//! \brief Construct a ROSVelocityPublisher object
			ROSVelocityPublisher()
			{
				ros::NodeHandle n;
				velocityPublisher = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1, true);
			}

			virtual void publish(const Velocity2D& command) const;

		private:

			//! The publisher itself
			ros::Publisher velocityPublisher;

	};
}

#endif
