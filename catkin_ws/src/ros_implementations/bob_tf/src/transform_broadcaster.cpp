#include "bob_tf/transform_broadcaster.h"

#include <tf2_msgs/TFMessage.h>

namespace bob 
{
	TransformBroadcaster::TransformBroadcaster()
	{
		publisher_ = node_.advertise<tf2_msgs::TFMessage>("/tf", 100);
	}

	void TransformBroadcaster::sendTransform(const geometry_msgs::TransformStamped & msgtf)
	{
		// TFMessage is designed to be able to hold many messages (ie for robotic arm)
		// We remove that and only support one message
		tf2_msgs::TFMessage message;
		message.transforms.push_back(msgtf);
		publisher_.publish(message);
	}

	void TransformBroadcaster::sendTransform(const StampedTransform & transform)
	{
		geometry_msgs::TransformStamped msgtf;
		transformStampedTFToMsg(transform, msgtf);
		sendTransform(msgtf);
	} 
}
