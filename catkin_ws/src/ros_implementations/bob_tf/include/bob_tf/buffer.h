#ifndef _BOB_TF_BUFFER_H_
#define _BOB_TF_BUFFER_H_

#include <bob_tf/buffer_core.h>
#include <tf2_msgs/FrameGraph.h>
#include <ros/ros.h>

namespace bob
{

	class Buffer : public BufferCore
	{
		public:
			using BufferCore::lookupTransform;
			using BufferCore::canTransform;

			Buffer(ros::Duration cache_time = ros::Duration(BufferCore::DEFAULT_CACHE_TIME), bool debug = false);

			virtual geometry_msgs::TransformStamped 
				lookupTransform(const std::string& target_frame, const std::string& source_frame,
						const ros::Time& time, const ros::Duration timeout) const;

			virtual geometry_msgs::TransformStamped 
				lookupTransform(const std::string& target_frame, const ros::Time& target_time,
						const std::string& source_frame, const ros::Time& source_time,
						const std::string& fixed_frame, const ros::Duration timeout) const;


			virtual bool
				canTransform(const std::string& target_frame, const std::string& source_frame, 
						const ros::Time& target_time, const ros::Duration timeout, std::string* errstr = NULL) const;

			virtual bool
				canTransform(const std::string& target_frame, const ros::Time& target_time,
						const std::string& source_frame, const ros::Time& source_time,
						const std::string& fixed_frame, const ros::Duration timeout, std::string* errstr = NULL) const;

	}; 
} 

#endif
