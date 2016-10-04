#ifndef _BOB_ROS_IMPLEMENTATIONS_ROS_BUMPER_HANDLE_H_
#define _BOB_ROS_IMPLEMENTATIONS_ROS_BUMPER_HANDLE_H_

#include <kobuki_msgs/BumperEvent.h>

#include <ros/ros.h>
#include <bob_sensor/ibumper_handle.h>

namespace bob
{

	class ROSBumperHandle : public IBumperHandle
	{


		public:

			ROSBumperHandle();

			virtual BumperState getState() const;

			virtual bool hitSomething() const;

			virtual void clearFlags();

		private: 

			enum Bumper
			{
				Left = 0,
				Center = 1,
				Right = 2
			};

			enum EventType
			{
				Pressed = 1,
				Released = 0
			};

			void messageCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);

			ros::Subscriber bumperSubscriber;

			bool lastBumperState[3];


	};

}

#endif
