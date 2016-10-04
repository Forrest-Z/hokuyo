#ifndef _BOB_TF_BOB_TRANSFORM_LISTENER_H_
#define _BOB_TF_BOB_TRANSFORM_LISTENER_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/thread.hpp>

#include <tf2_msgs/TFMessage.h>
#include <tf/transform_listener.h>

#include <bob_tf/buffer.h>

#include <boost/thread.hpp>

#include <tf/transform_datatypes.h>

using namespace tf;
namespace bob
{

	/** strip a leading slash for */
	std::string strip_leading_slash(const std::string& frame_name);

	class BobTransformListener : public tf::TransformListener 
	{

		public:

			BobTransformListener(ros::Duration max_cache_time = ros::Duration(10.0), bool spin_thread = true);

			BobTransformListener(const ros::NodeHandle& nh,
					ros::Duration max_cache_time = ros::Duration(10.0), bool spin_thread = true);

			virtual void lookupTransform(const std::string& target_frame, const ros::Time& target_time,
					const std::string& source_frame, const ros::Time& source_time,
					const std::string& fixed_frame, StampedTransform& transform) const;

			virtual bool waitForTransform(const std::string& target_frame, const ros::Time& target_time,
					const std::string& source_frame, const ros::Time& source_time,
					const std::string& fixed_frame,
					const ros::Duration& timeout, const ros::Duration& polling_sleep_duration = ros::Duration(0.01),
					std::string* error_msg = NULL) const;

			virtual void transformPose(const std::string& target_frame, const tf::Stamped<tf::Pose>& stamped_in, tf::Stamped<tf::Pose>& stamped_out) const;

			virtual bool canTransform(const std::string& target_frame, const std::string& source_frame,
					const ros::Time& time,
					std::string* error_msg = NULL) const;

			virtual void lookupTransform(const std::string& target_frame, const std::string& source_frame,
					const ros::Time& time, StampedTransform& transform) const;

			virtual bool waitForTransform(const std::string& target_frame, const std::string& source_frame,
					const ros::Time& time,
					const ros::Duration& timeout, const ros::Duration& polling_sleep_duration,
					std::string* error_msg) const;
			virtual void transformPoint(const std::string& target_frame, const tf::Stamped<Point>& stamped_in, tf::Stamped<Point>& stamped_out) const;
		private:

			void initWithThread();	

			ros::NodeHandle node_;

			ros::CallbackQueue tf_message_callback_queue_;
			boost::thread dedicated_listener_thread_;
			ros::Subscriber message_subscriber_tf_;
			ros::Time last_update_;

			void subscription_callback(const ros::MessageEvent<tf2_msgs::TFMessage const>& msg_evt);
			void subscription_callback_impl(const ros::MessageEvent<tf2_msgs::TFMessage const>& msg_evt, bool is_static);

			void dedicatedListenerThread()
			{
				while (ros::ok())
				{
					tf_message_callback_queue_.callAvailable(ros::WallDuration(0.01));
				}
			};

			Buffer tf2_buffer_;


	};
}

#endif 
