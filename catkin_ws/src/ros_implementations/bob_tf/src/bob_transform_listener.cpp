#include <bob_tf/bob_transform_listener.h>

#include <tf/exceptions.h>

using namespace tf;

namespace bob
{
	std::string strip_leading_slash(const std::string& frame_name)
	{
		if (frame_name.size() > 0)
			if (frame_name[0] == '/')
			{
				std::string shorter = frame_name;
				shorter.erase(0,1);
				return shorter;
			}

		return frame_name;
	}

	BobTransformListener::BobTransformListener(ros::Duration max_cache_time, bool spin_thread) : 
		tf2_buffer_(max_cache_time)
	{
		initWithThread();
	}

	BobTransformListener::BobTransformListener(const ros::NodeHandle& nh, ros::Duration max_cache_time, bool spin_thread) :
		node_(nh),
		tf2_buffer_(max_cache_time)
	{
		initWithThread();
	}


	void BobTransformListener::initWithThread()
	{
		ros::SubscribeOptions ops_tf = ros::SubscribeOptions::create<tf2_msgs::TFMessage>("/tf", 100, boost::bind(&BobTransformListener::subscription_callback, this, _1), ros::VoidPtr(), &tf_message_callback_queue_);
		message_subscriber_tf_ = node_.subscribe(ops_tf);

		dedicated_listener_thread_ = boost::thread(boost::bind(&BobTransformListener::dedicatedListenerThread, this));

		//Tell the buffer we have a dedicated thread to enable timeouts
		tf2_buffer_.setUsingDedicatedThread(true);
	}

	bool BobTransformListener::canTransform(const std::string& target_frame, const std::string& source_frame,
			const ros::Time& time, std::string* error_msg) const
	{
		return tf2_buffer_.canTransform(strip_leading_slash(target_frame),
				strip_leading_slash(source_frame), time, error_msg);
	}


	bool BobTransformListener::waitForTransform(const std::string& target_frame,const ros::Time& target_time, const std::string& source_frame,
			const ros::Time& source_time, const std::string& fixed_frame,
			const ros::Duration& timeout, const ros::Duration& polling_sleep_duration,
			std::string* error_msg) const
	{
		return tf2_buffer_.canTransform(strip_leading_slash(target_frame), target_time,
				strip_leading_slash(source_frame), source_time,
				strip_leading_slash(fixed_frame), timeout, error_msg);
	}

	void BobTransformListener::transformPose(const std::string& target_frame, const tf::Stamped<Pose>& stamped_in, tf::Stamped<Pose>& stamped_out) const
	{
		StampedTransform transform;
		lookupTransform(target_frame, stamped_in.frame_id_, stamped_in.stamp_, transform);

		stamped_out.setData(transform * stamped_in);
		stamped_out.stamp_ = transform.stamp_;
		stamped_out.frame_id_ = target_frame;
	}

	void BobTransformListener::lookupTransform(const std::string& target_frame,const ros::Time& target_time, const std::string& source_frame,
			const ros::Time& source_time, const std::string& fixed_frame, StampedTransform& transform) const
	{
		geometry_msgs::TransformStamped output = 
			tf2_buffer_.lookupTransform(strip_leading_slash(target_frame), target_time,
					strip_leading_slash(source_frame), source_time,
					strip_leading_slash(fixed_frame));
		transformStampedMsgToTF(output, transform);
	}

	void BobTransformListener::lookupTransform(const std::string& target_frame, const std::string& source_frame,
			const ros::Time& time, StampedTransform& transform) const
	{
		geometry_msgs::TransformStamped output = 
			tf2_buffer_.lookupTransform(strip_leading_slash(target_frame),
					strip_leading_slash(source_frame), time);
		transformStampedMsgToTF(output, transform);
		return;
	};

	bool BobTransformListener::waitForTransform(const std::string& target_frame, const std::string& source_frame,
			const ros::Time& time,
			const ros::Duration& timeout, const ros::Duration& polling_sleep_duration,
			std::string* error_msg) const
	{
		return tf2_buffer_.canTransform(strip_leading_slash(target_frame),
				strip_leading_slash(source_frame), time, timeout, error_msg);
	}
	void BobTransformListener::subscription_callback(const ros::MessageEvent<tf2_msgs::TFMessage const>& msg_evt)
	{
		subscription_callback_impl(msg_evt, false);
	}

	void BobTransformListener::subscription_callback_impl(const ros::MessageEvent<tf2_msgs::TFMessage const>& msg_evt, bool is_static)
	{
		ros::Time now = ros::Time::now();
		if(now < last_update_){
			ROS_WARN("Detected jump back in time. Clearing TF buffer.");
			tf2_buffer_.clear();
		}
		last_update_ = now;



		const tf2_msgs::TFMessage& msg_in = *(msg_evt.getConstMessage());
		std::string authority = msg_evt.getPublisherName(); // lookup the authority
		for (unsigned int i = 0; i < msg_in.transforms.size(); i++)
		{
			try
			{
				tf2_buffer_.setTransform(msg_in.transforms[i], authority, is_static);
			}

			catch (tf2::TransformException& ex)
			{
				///\todo Use error reporting
				std::string temp = ex.what();
				ROS_ERROR("Failure to set recieved transform from %s to %s with error: %s\n", msg_in.transforms[i].child_frame_id.c_str(), msg_in.transforms[i].header.frame_id.c_str(), temp.c_str());
			}
		}
	}
	void BobTransformListener::transformPoint(const std::string& target_frame, const tf::Stamped<Point>& stamped_in, tf::Stamped<Point>& stamped_out) const
	{
		StampedTransform transform;
		lookupTransform(target_frame, stamped_in.frame_id_, stamped_in.stamp_, transform);

		stamped_out.setData(transform * stamped_in);
		stamped_out.stamp_ = transform.stamp_;
		stamped_out.frame_id_ = target_frame;
	}

}
