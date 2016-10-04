#ifndef _BOB_ROS_IMPLEMENTATIONS_ROS_ODOMETRY_HANDLE_H_
#define _BOB_ROS_IMPLEMENTATIONS_ROS_ODOMETRY_HANDLE_H_

#include <bob_tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <bob_toolbox/velocity2d.h>
#include <bob_sensor/iodometry_handle.h>

namespace bob 
{

	class ROSOdometryHandle : public IOdometryHandle
	{
		public:

			ROSOdometryHandle();

			void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

			virtual Velocity2D getRobotVel() const;

		private:

			ros::Subscriber odomSubscriber;

			nav_msgs::Odometry odomData;

	};

} 

#endif 
