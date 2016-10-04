#include "bob_tf/buffer.h"

#include <ros/assert.h>
#include <sstream>

namespace bob
{

	Buffer::Buffer(ros::Duration cache_time, bool debug) :
		BufferCore(cache_time)
	{
		if(debug && !ros::service::exists("~tf2_frames", false))
		{
			ros::NodeHandle n("~");
		}
	}

	geometry_msgs::TransformStamped 
		Buffer::lookupTransform(const std::string& target_frame, const std::string& source_frame,
				const ros::Time& time, const ros::Duration timeout) const
		{
			canTransform(target_frame, source_frame, time, timeout);
			return lookupTransform(target_frame, source_frame, time);
		}


	geometry_msgs::TransformStamped 
		Buffer::lookupTransform(const std::string& target_frame, const ros::Time& target_time,
				const std::string& source_frame, const ros::Time& source_time,
				const std::string& fixed_frame, const ros::Duration timeout) const
		{
			canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, timeout);
			return lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame);
		}

	/** This is a workaround for the case that we're running inside of
	  rospy and ros::Time is not initialized inside the c++ instance. 
	  This makes the system fall back to Wall time if not initialized.  
	 */
	ros::Time now_fallback_to_wall()
	{
		try
		{
			return ros::Time::now();
		}
		catch (ros::TimeNotInitializedException ex)
		{
			ros::WallTime wt = ros::WallTime::now(); 
			return ros::Time(wt.sec, wt.nsec); 
		}
	}

	/** This is a workaround for the case that we're running inside of
	  rospy and ros::Time is not initialized inside the c++ instance. 
	  This makes the system fall back to Wall time if not initialized.  
https://github.com/ros/geometry/issues/30
	 */
	void sleep_fallback_to_wall(const ros::Duration& d)
	{
		try
		{
			d.sleep();
		}
		catch (ros::TimeNotInitializedException ex)
		{
			ros::WallDuration wd = ros::WallDuration(d.sec, d.nsec); 
			wd.sleep();
		}
	}

	void conditionally_append_timeout_info(std::string * errstr, const ros::Time& start_time,
			const ros::Duration& timeout)
	{
		if (errstr)
		{
			std::stringstream ss;
			ss << ". canTransform returned after "<< (now_fallback_to_wall() - start_time).toSec() \
				<<" timeout was " << timeout.toSec() << ".";
			(*errstr) += ss.str();
		}
	}

	bool
		Buffer::canTransform(const std::string& target_frame, const std::string& source_frame, 
				const ros::Time& time, const ros::Duration timeout, std::string* errstr) const
		{
			// poll for transform if timeout is set
			ros::Time start_time = now_fallback_to_wall();
			while (now_fallback_to_wall() < start_time + timeout && 
					!canTransform(target_frame, source_frame, time) &&
					(now_fallback_to_wall()+ros::Duration(3.0) >= start_time) &&  //don't wait when we detect a bag loop
					(ros::ok() || !ros::isInitialized())) // Make sure we haven't been stopped (won't work for pytf)
					{
						sleep_fallback_to_wall(ros::Duration(0.01));
					}
			bool retval = canTransform(target_frame, source_frame, time, errstr);
			conditionally_append_timeout_info(errstr, start_time, timeout);
			return retval;
		}


	bool
		Buffer::canTransform(const std::string& target_frame, const ros::Time& target_time,
				const std::string& source_frame, const ros::Time& source_time,
				const std::string& fixed_frame, const ros::Duration timeout, std::string* errstr) const
		{
			// poll for transform if timeout is set
			ros::Time start_time = now_fallback_to_wall();
			while (now_fallback_to_wall() < start_time + timeout && 
					!canTransform(target_frame, target_time, source_frame, source_time, fixed_frame) &&
					(now_fallback_to_wall()+ros::Duration(3.0) >= start_time) &&  //don't wait when we detect a bag loop
					(ros::ok() || !ros::isInitialized())) // Make sure we haven't been stopped (won't work for pytf)
					{  
						sleep_fallback_to_wall(ros::Duration(0.01));
					}
			bool retval = canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, errstr);
			conditionally_append_timeout_info(errstr, start_time, timeout);
			return retval; 
		}

}
