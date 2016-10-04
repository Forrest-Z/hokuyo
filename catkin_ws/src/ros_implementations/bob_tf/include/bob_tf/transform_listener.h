#ifndef _BOB_TF_TRANSFORM_LISTENER_H_
#define _BOB_TF_TRANSFORM_LISTENER_H_

#include <boost/thread.hpp>

#include <bob_tf/buffer.h>

#include <bob_tf/transform_datatypes.h>

#include <ros/callback_queue.h>
#include <tf2_msgs/TFMessage.h>

namespace bob
{

	class TransformListener
	{ 

		public:

			TransformListener(ros::Duration max_cache_time = ros::Duration(10.0));

			std::string strip_leading_slash(const std::string& frame_name) const;

			void lookupTransform(const std::string& target_frame, const std::string& source_frame,
					const ros::Time& time, StampedTransform& transform) const;

			void lookupTransform(const std::string& target_frame, const ros::Time& target_time,
					const std::string& source_frame, const ros::Time& source_time,
					const std::string& fixed_frame, StampedTransform& transform) const;

			bool waitForTransform(const std::string& target_frame, const std::string& source_frame,
					const ros::Time& time, const ros::Duration& timeout, const ros::Duration& polling_sleep_duration = ros::Duration(0.01),
					std::string* error_msg = NULL) const;

			bool waitForTransform(const std::string& target_frame, const ros::Time& target_time,
					const std::string& source_frame, const ros::Time& source_time,
					const std::string& fixed_frame,
					const ros::Duration& timeout, const ros::Duration& polling_sleep_duration = ros::Duration(0.01),
					std::string* error_msg = NULL) const;

			void transformPoint(const std::string& target_frame, const Stamped<Point>& stamped_in, Stamped<Point>& stamped_out) const;

			void transformPose(const std::string& target_frame, const Stamped<Pose>& stamped_in, Stamped<Pose>& stamped_out) const;

			void transformPose(const std::string& target_frame, const ros::Time& target_time,
					const Stamped<Pose>& stamped_in,
					const std::string& fixed_frame,
					Stamped<Pose>& stamped_out) const;

		private:

			void initWithThread();

			///! Callback function for ros message subscriptoin
			void subscription_callback(const ros::MessageEvent<tf2_msgs::TFMessage const>& msg_evt);
			void subscription_callback_impl(const ros::MessageEvent<tf2_msgs::TFMessage const>& msg_evt, bool is_static);

			void dedicatedListenerThread()
			{
				while (ros::ok())
				{
					tf_message_callback_queue_.callAvailable(ros::WallDuration(0.01));
				}
			}

			ros::NodeHandle node_;

			Buffer tf2_buffer_;


			ros::CallbackQueue tf_message_callback_queue_;
			boost::thread dedicated_listener_thread_;
			ros::Subscriber message_subscriber_tf_;
			ros::Time last_update_;
	};

}

#endif
