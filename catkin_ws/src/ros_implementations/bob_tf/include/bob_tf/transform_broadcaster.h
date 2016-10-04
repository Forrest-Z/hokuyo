#ifndef _BOB_TF_TRANSFORM_BROADCASTER_H_
#define _BOB_TF_TRANSFORM_BROADCASTER_H_

#include <ros/ros.h>

#include <bob_tf/transform_datatypes.h>

namespace bob
{

	class TransformBroadcaster
	{

		public:

			TransformBroadcaster();

			void sendTransform(const StampedTransform & transform);

			void sendTransform(const geometry_msgs::TransformStamped & transform);

		private:

			ros::NodeHandle node_;
			ros::Publisher publisher_;

	};

}

#endif
