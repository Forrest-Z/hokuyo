#include "bob_tf/buffer_core.h"
#include "bob_tf/time_cache.h"
#include "bob_tf/exceptions.h"
#include "tf2_msgs/TF2Error.h"

#include <assert.h>
#include <console_bridge/console.h>
#include "bob_tf/LinearMath/Transform.h"

namespace bob
{

	/** \brief convert Transform msg to Transform */
	void transformMsgToTF2(const geometry_msgs::Transform& msg, Transform& tf2)
	{tf2 = Transform(Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w), Vector3(msg.translation.x, msg.translation.y, msg.translation.z));}

	/** \brief convert Transform to Transform msg*/
	void transformTF2ToMsg(const Transform& tf2, geometry_msgs::Transform& msg)
	{
		msg.translation.x = tf2.getOrigin().x();
		msg.translation.y = tf2.getOrigin().y();
		msg.translation.z = tf2.getOrigin().z();
		msg.rotation.x = tf2.getRotation().x();
		msg.rotation.y = tf2.getRotation().y();
		msg.rotation.z = tf2.getRotation().z();
		msg.rotation.w = tf2.getRotation().w();
	}

	/** \brief convert Transform to Transform msg*/
	void transformTF2ToMsg(const Transform& tf2, geometry_msgs::TransformStamped& msg, ros::Time stamp, const std::string& frame_id, const std::string& child_frame_id)
	{
		transformTF2ToMsg(tf2, msg.transform);
		msg.header.stamp = stamp;
		msg.header.frame_id = frame_id;
		msg.child_frame_id = child_frame_id;
	}

	void transformTF2ToMsg(const Quaternion& orient, const Vector3& pos, geometry_msgs::Transform& msg)
	{
		msg.translation.x = pos.x();
		msg.translation.y = pos.y();
		msg.translation.z = pos.z();
		msg.rotation.x = orient.x();
		msg.rotation.y = orient.y();
		msg.rotation.z = orient.z();
		msg.rotation.w = orient.w();
	}

	void transformTF2ToMsg(const Quaternion& orient, const Vector3& pos, geometry_msgs::TransformStamped& msg, ros::Time stamp, const std::string& frame_id, const std::string& child_frame_id)
	{
		transformTF2ToMsg(orient, pos, msg.transform);
		msg.header.stamp = stamp;
		msg.header.frame_id = frame_id;
		msg.child_frame_id = child_frame_id;
	}

	void setIdentity(geometry_msgs::Transform& tx)
	{
		tx.translation.x = 0;
		tx.translation.y = 0;
		tx.translation.z = 0;
		tx.rotation.x = 0;
		tx.rotation.y = 0;
		tx.rotation.z = 0;
		tx.rotation.w = 1;
	}

	bool startsWithSlash(const std::string& frame_id)
	{
		if (frame_id.size() > 0)
			if (frame_id[0] == '/')
				return true;
		return false;
	}

	std::string stripSlash(const std::string& in)
	{
		std::string out = in;
		if (startsWithSlash(in))
			out.erase(0,1);
		return out;
	}


	bool BufferCore::warnFrameId(const char* function_name_arg, const std::string& frame_id) const
	{
		if (frame_id.size() == 0)
		{
			std::stringstream ss;
			ss << "Invalid argument passed to "<< function_name_arg <<" in tf2 frame_ids cannot be empty";
			logWarn("%s",ss.str().c_str());
			return true;
		}

		if (startsWithSlash(frame_id))
		{
			std::stringstream ss;
			ss << "Invalid argument \"" << frame_id << "\" passed to "<< function_name_arg <<" in tf2 frame_ids cannot start with a '/' like: ";
			logWarn("%s",ss.str().c_str());
			return true;
		}

		return false;
	}

	CompactFrameID BufferCore::validateFrameId(const char* function_name_arg, const std::string& frame_id) const
	{
		if (frame_id.empty())
		{
			std::stringstream ss;
			ss << "Invalid argument passed to "<< function_name_arg <<" in tf2 frame_ids cannot be empty";
			throw InvalidArgumentException(ss.str().c_str());
		}

		if (startsWithSlash(frame_id))
		{
			std::stringstream ss;
			ss << "Invalid argument \"" << frame_id << "\" passed to "<< function_name_arg <<" in tf2 frame_ids cannot start with a '/' like: ";
			throw InvalidArgumentException(ss.str().c_str());
		}

		CompactFrameID id = lookupFrameNumber(frame_id);
		if (id == 0)
		{
			std::stringstream ss;
			ss << "\"" << frame_id << "\" passed to "<< function_name_arg <<" does not exist. ";
			throw LookupException(ss.str().c_str());
		}

		return id;
	}

	BufferCore::BufferCore(ros::Duration cache_time)
		: cache_time_(cache_time)
		  , transformable_callbacks_counter_(0)
		  , transformable_requests_counter_(0)
	{
		frameIDs_["NO_PARENT"] = 0;
		frames_.push_back(TimeCacheInterfacePtr());
		frameIDs_reverse.push_back("NO_PARENT");
	}

	void BufferCore::clear()
	{
		//old_tf_.clear();


		boost::mutex::scoped_lock lock(frame_mutex_);
		if ( frames_.size() > 1 )
		{
			for (std::vector<TimeCacheInterfacePtr>::iterator  cache_it = frames_.begin() + 1; cache_it != frames_.end(); ++cache_it)
			{
				if (*cache_it)
					(*cache_it)->clearList();
			}
		}

	}

	bool BufferCore::setTransform(const geometry_msgs::TransformStamped& transform_in, const std::string& authority, bool is_static)
	{
		geometry_msgs::TransformStamped stripped = transform_in;
		stripped.header.frame_id = stripSlash(stripped.header.frame_id);
		stripped.child_frame_id = stripSlash(stripped.child_frame_id);


		bool error_exists = false;
		if (stripped.child_frame_id == stripped.header.frame_id)
		{
			logError("TF_SELF_TRANSFORM: Ignoring transform from authority \"%s\" with frame_id and child_frame_id  \"%s\" because they are the same",  authority.c_str(), stripped.child_frame_id.c_str());
			error_exists = true;
		}

		if (stripped.child_frame_id == "")
		{
			logError("TF_NO_CHILD_FRAME_ID: Ignoring transform from authority \"%s\" because child_frame_id not set ", authority.c_str());
			error_exists = true;
		}

		if (stripped.header.frame_id == "")
		{
			logError("TF_NO_FRAME_ID: Ignoring transform with child_frame_id \"%s\"  from authority \"%s\" because frame_id not set", stripped.child_frame_id.c_str(), authority.c_str());
			error_exists = true;
		}

		if (std::isnan(stripped.transform.translation.x) || std::isnan(stripped.transform.translation.y) || std::isnan(stripped.transform.translation.z)||
				std::isnan(stripped.transform.rotation.x) ||       std::isnan(stripped.transform.rotation.y) ||       std::isnan(stripped.transform.rotation.z) ||       std::isnan(stripped.transform.rotation.w))
		{
			logError("TF_NAN_INPUT: Ignoring transform for child_frame_id \"%s\" from authority \"%s\" because of a nan value in the transform (%f %f %f) (%f %f %f %f)",
					stripped.child_frame_id.c_str(), authority.c_str(),
					stripped.transform.translation.x, stripped.transform.translation.y, stripped.transform.translation.z,
					stripped.transform.rotation.x, stripped.transform.rotation.y, stripped.transform.rotation.z, stripped.transform.rotation.w
				);
			error_exists = true;
		}

		if (error_exists)
			return false;

		{
			boost::mutex::scoped_lock lock(frame_mutex_);
			CompactFrameID frame_number = lookupOrInsertFrameNumber(stripped.child_frame_id);
			TimeCacheInterfacePtr frame = getFrame(frame_number);
			if (frame == NULL)
				frame = allocateFrame(frame_number, is_static);

			if (frame->insertData(TransformStorage(stripped, lookupOrInsertFrameNumber(stripped.header.frame_id), frame_number)))
			{
				frame_authority_[frame_number] = authority;
			}
			else
			{
				logWarn("TF_OLD_DATA ignoring data from the past for frame %s at time %g according to authority %s\nPossible reasons are listed at http://wiki.ros.org/tf/Errors%%20explained", stripped.child_frame_id.c_str(), stripped.header.stamp.toSec(), authority.c_str());
				return false;
			}
		}

		testTransformableRequests();

		return true;
	}

	TimeCacheInterfacePtr BufferCore::allocateFrame(CompactFrameID cfid, bool is_static)
	{
		TimeCacheInterfacePtr frame_ptr = frames_[cfid];
		if (is_static) {
			frames_[cfid] = TimeCacheInterfacePtr(new StaticCache());
		} else {
			frames_[cfid] = TimeCacheInterfacePtr(new TimeCache(cache_time_));
		}

		return frames_[cfid];
	}

	enum WalkEnding
	{
		Identity,
		TargetParentOfSource,
		SourceParentOfTarget,
		FullPath,
	};

	// TODO for Jade: Merge walkToTopParent functions; this is now a stub to preserve ABI
	template<typename F>
		int BufferCore::walkToTopParent(F& f, ros::Time time, CompactFrameID target_id, CompactFrameID source_id, std::string* error_string) const
		{
			return walkToTopParent(f, time, target_id, source_id, error_string, NULL);
		}

	template<typename F>
		int BufferCore::walkToTopParent(F& f, ros::Time time, CompactFrameID target_id,
				CompactFrameID source_id, std::string* error_string, std::vector<CompactFrameID>
				*frame_chain) const
		{
			if (frame_chain)
				frame_chain->clear();

			// Short circuit if zero length transform to allow lookups on non existant links
			if (source_id == target_id)
			{
				f.finalize(Identity, time);
				return tf2_msgs::TF2Error::NO_ERROR;
			}

			//If getting the latest get the latest common time
			if (time == ros::Time())
			{
				int retval = getLatestCommonTime(target_id, source_id, time, error_string);
				if (retval != tf2_msgs::TF2Error::NO_ERROR)
				{
					return retval;
				}
			}

			// Walk the tree to its root from the source frame, accumulating the transform
			CompactFrameID frame = source_id;
			CompactFrameID top_parent = frame;
			uint32_t depth = 0;

			std::string extrapolation_error_string;
			bool extrapolation_might_have_occurred = false;

			while (frame != 0)
			{
				TimeCacheInterfacePtr cache = getFrame(frame);
				if (frame_chain)
					frame_chain->push_back(frame);

				if (!cache)
				{
					// There will be no cache for the very root of the tree
					top_parent = frame;
					break;
				}

				CompactFrameID parent = f.gather(cache, time, &extrapolation_error_string);
				if (parent == 0)
				{
					// Just break out here... there may still be a path from source -> target
					top_parent = frame;
					extrapolation_might_have_occurred = true;
					break;
				}

				// Early out... target frame is a direct parent of the source frame
				if (frame == target_id)
				{
					f.finalize(TargetParentOfSource, time);
					return tf2_msgs::TF2Error::NO_ERROR;
				}

				f.accum(true);

				top_parent = frame;
				frame = parent;

				++depth;
				if (depth > MAX_GRAPH_DEPTH)
				{
					if (error_string)
					{
						std::stringstream ss;
						ss << "The tf tree is invalid because it contains a loop." << std::endl
							<< allFramesAsStringNoLock() << std::endl;
						*error_string = ss.str();
					}
					return tf2_msgs::TF2Error::LOOKUP_ERROR;
				}
			}

			// Now walk to the top parent from the target frame, accumulating its transform
			frame = target_id;
			depth = 0;
			std::vector<CompactFrameID> reverse_frame_chain;

			while (frame != top_parent)
			{
				TimeCacheInterfacePtr cache = getFrame(frame);
				if (frame_chain)
					reverse_frame_chain.push_back(frame);

				if (!cache)
				{
					break;
				}

				CompactFrameID parent = f.gather(cache, time, error_string);
				if (parent == 0)
				{
					if (error_string)
					{
						std::stringstream ss;
						ss << *error_string << ", when looking up transform from frame [" << lookupFrameString(source_id) << "] to frame [" << lookupFrameString(target_id) << "]";
						*error_string = ss.str();
					}

					return tf2_msgs::TF2Error::EXTRAPOLATION_ERROR;
				}

				// Early out... source frame is a direct parent of the target frame
				if (frame == source_id)
				{
					f.finalize(SourceParentOfTarget, time);
					if (frame_chain)
					{
						frame_chain->swap(reverse_frame_chain);
					}
					return tf2_msgs::TF2Error::NO_ERROR;
				}

				f.accum(false);

				frame = parent;

				++depth;
				if (depth > MAX_GRAPH_DEPTH)
				{
					if (error_string)
					{
						std::stringstream ss;
						ss << "The tf tree is invalid because it contains a loop." << std::endl
							<< allFramesAsStringNoLock() << std::endl;
						*error_string = ss.str();
					}
					return tf2_msgs::TF2Error::LOOKUP_ERROR;
				}
			}

			if (frame != top_parent)
			{
				if (extrapolation_might_have_occurred)
				{
					if (error_string)
					{
						std::stringstream ss;
						ss << extrapolation_error_string << ", when looking up transform from frame [" << lookupFrameString(source_id) << "] to frame [" << lookupFrameString(target_id) << "]";
						*error_string = ss.str();
					}

					return tf2_msgs::TF2Error::EXTRAPOLATION_ERROR;

				}

				createConnectivityErrorString(source_id, target_id, error_string);
				return tf2_msgs::TF2Error::CONNECTIVITY_ERROR;
			}

			f.finalize(FullPath, time);
			if (frame_chain)
			{
				// Pruning: Compare the chains starting at the parent (end) until they differ
				int m = reverse_frame_chain.size()-1;
				int n = frame_chain->size()-1;
				for (; m >= 0 && n >= 0; --m, --n)
				{
					if ((*frame_chain)[n] != reverse_frame_chain[m])
						break;
				}
				// Erase all duplicate items from frame_chain
				if (n > 0)
					frame_chain->erase(frame_chain->begin() + (n-1), frame_chain->end());

				if (m < reverse_frame_chain.size())
				{
					for (int i = m; i >= 0; --i)
					{
						frame_chain->push_back(reverse_frame_chain[i]);
					}
				}
			}

			return tf2_msgs::TF2Error::NO_ERROR;
		}



	struct TransformAccum
	{
		TransformAccum()
			: source_to_top_quat(0.0, 0.0, 0.0, 1.0)
			  , source_to_top_vec(0.0, 0.0, 0.0)
			  , target_to_top_quat(0.0, 0.0, 0.0, 1.0)
			  , target_to_top_vec(0.0, 0.0, 0.0)
			  , result_quat(0.0, 0.0, 0.0, 1.0)
			  , result_vec(0.0, 0.0, 0.0)
		{
		}

		CompactFrameID gather(TimeCacheInterfacePtr cache, ros::Time time, std::string* error_string)
		{
			if (!cache->getData(time, st, error_string))
			{
				return 0;
			}

			return st.frame_id_;
		}

		void accum(bool source)
		{
			if (source)
			{
				source_to_top_vec = quatRotate(st.rotation_, source_to_top_vec) + st.translation_;
				source_to_top_quat = st.rotation_ * source_to_top_quat;
			}
			else
			{
				target_to_top_vec = quatRotate(st.rotation_, target_to_top_vec) + st.translation_;
				target_to_top_quat = st.rotation_ * target_to_top_quat;
			}
		}

		void finalize(WalkEnding end, ros::Time _time)
		{
			switch (end)
			{
				case Identity:
					break;
				case TargetParentOfSource:
					result_vec = source_to_top_vec;
					result_quat = source_to_top_quat;
					break;
				case SourceParentOfTarget:
					{
						Quaternion inv_target_quat = target_to_top_quat.inverse();
						Vector3 inv_target_vec = quatRotate(inv_target_quat, -target_to_top_vec);
						result_vec = inv_target_vec;
						result_quat = inv_target_quat;
						break;
					}
				case FullPath:
					{
						Quaternion inv_target_quat = target_to_top_quat.inverse();
						Vector3 inv_target_vec = quatRotate(inv_target_quat, -target_to_top_vec);

						result_vec = quatRotate(inv_target_quat, source_to_top_vec) + inv_target_vec;
						result_quat = inv_target_quat * source_to_top_quat;
					}
					break;
			};

			time = _time;
		}

		TransformStorage st;
		ros::Time time;
		Quaternion source_to_top_quat;
		Vector3 source_to_top_vec;
		Quaternion target_to_top_quat;
		Vector3 target_to_top_vec;

		Quaternion result_quat;
		Vector3 result_vec;
	};

	geometry_msgs::TransformStamped BufferCore::lookupTransform(const std::string& target_frame,
			const std::string& source_frame,
			const ros::Time& time) const
	{
		boost::mutex::scoped_lock lock(frame_mutex_);

		if (target_frame == source_frame) {
			geometry_msgs::TransformStamped identity;
			identity.header.frame_id = target_frame;
			identity.child_frame_id = source_frame;
			identity.transform.rotation.w = 1;

			if (time == ros::Time())
			{
				CompactFrameID target_id = lookupFrameNumber(target_frame);
				TimeCacheInterfacePtr cache = getFrame(target_id);
				if (cache)
					identity.header.stamp = cache->getLatestTimestamp();
				else
					identity.header.stamp = time;
			}
			else
				identity.header.stamp = time;

			return identity;
		}

		//Identify case does not need to be validated above
		CompactFrameID target_id = validateFrameId("lookupTransform argument target_frame", target_frame);
		CompactFrameID source_id = validateFrameId("lookupTransform argument source_frame", source_frame);

		std::string error_string;
		TransformAccum accum;
		int retval = walkToTopParent(accum, time, target_id, source_id, &error_string);
		if (retval != tf2_msgs::TF2Error::NO_ERROR)
		{
			switch (retval)
			{
				case tf2_msgs::TF2Error::CONNECTIVITY_ERROR:
					throw ConnectivityException(error_string);
				case tf2_msgs::TF2Error::EXTRAPOLATION_ERROR:
					throw ExtrapolationException(error_string);
				case tf2_msgs::TF2Error::LOOKUP_ERROR:
					throw LookupException(error_string);
				default:
					logError("Unknown error code: %d", retval);
					assert(0);
			}
		}

		geometry_msgs::TransformStamped output_transform;
		transformTF2ToMsg(accum.result_quat, accum.result_vec, output_transform, accum.time, target_frame, source_frame);
		return output_transform;
	}


	geometry_msgs::TransformStamped BufferCore::lookupTransform(const std::string& target_frame, 
			const ros::Time& target_time,
			const std::string& source_frame,
			const ros::Time& source_time,
			const std::string& fixed_frame) const
	{
		validateFrameId("lookupTransform argument target_frame", target_frame);
		validateFrameId("lookupTransform argument source_frame", source_frame);
		validateFrameId("lookupTransform argument fixed_frame", fixed_frame);

		geometry_msgs::TransformStamped output;
		geometry_msgs::TransformStamped temp1 =  lookupTransform(fixed_frame, source_frame, source_time);
		geometry_msgs::TransformStamped temp2 =  lookupTransform(target_frame, fixed_frame, target_time);

		Transform tf1, tf2;
		transformMsgToTF2(temp1.transform, tf1);
		transformMsgToTF2(temp2.transform, tf2);
		transformTF2ToMsg(tf2*tf1, output.transform);
		output.header.stamp = temp2.header.stamp;
		output.header.frame_id = target_frame;
		output.child_frame_id = source_frame;
		return output;
	}

	struct CanTransformAccum
	{
		CompactFrameID gather(TimeCacheInterfacePtr cache, ros::Time time, std::string* error_string)
		{
			return cache->getParent(time, error_string);
		}

		void accum(bool source)
		{
		}

		void finalize(WalkEnding end, ros::Time _time)
		{
		}

		TransformStorage st;
	};

	bool BufferCore::canTransformNoLock(CompactFrameID target_id, CompactFrameID source_id,
			const ros::Time& time, std::string* error_msg) const
	{
		if (target_id == 0 || source_id == 0)
		{
			return false;
		}

		if (target_id == source_id)
		{
			return true;
		}

		CanTransformAccum accum;
		if (walkToTopParent(accum, time, target_id, source_id, error_msg) == tf2_msgs::TF2Error::NO_ERROR)
		{
			return true;
		}

		return false;
	}

	bool BufferCore::canTransformInternal(CompactFrameID target_id, CompactFrameID source_id,
			const ros::Time& time, std::string* error_msg) const
	{
		boost::mutex::scoped_lock lock(frame_mutex_);
		return canTransformNoLock(target_id, source_id, time, error_msg);
	}

	bool BufferCore::canTransform(const std::string& target_frame, const std::string& source_frame,
			const ros::Time& time, std::string* error_msg) const
	{
		// Short circuit if target_frame == source_frame
		if (target_frame == source_frame)
			return true;

		if (warnFrameId("canTransform argument target_frame", target_frame))
			return false;
		if (warnFrameId("canTransform argument source_frame", source_frame))
			return false;

		boost::mutex::scoped_lock lock(frame_mutex_);

		CompactFrameID target_id = lookupFrameNumber(target_frame);
		CompactFrameID source_id = lookupFrameNumber(source_frame);

		return canTransformNoLock(target_id, source_id, time, error_msg);
	}

	bool BufferCore::canTransform(const std::string& target_frame, const ros::Time& target_time,
			const std::string& source_frame, const ros::Time& source_time,
			const std::string& fixed_frame, std::string* error_msg) const
	{
		if (warnFrameId("canTransform argument target_frame", target_frame))
			return false;
		if (warnFrameId("canTransform argument source_frame", source_frame))
			return false;
		if (warnFrameId("canTransform argument fixed_frame", fixed_frame))
			return false;

		return canTransform(target_frame, fixed_frame, target_time) && canTransform(fixed_frame, source_frame, source_time, error_msg);
	}


	TimeCacheInterfacePtr BufferCore::getFrame(CompactFrameID frame_id) const
	{
		if (frame_id >= frames_.size())
			return TimeCacheInterfacePtr();
		else
		{
			return frames_[frame_id];
		}
	}

	CompactFrameID BufferCore::lookupFrameNumber(const std::string& frameid_str) const
	{
		CompactFrameID retval;
		M_StringToCompactFrameID::const_iterator map_it = frameIDs_.find(frameid_str);
		if (map_it == frameIDs_.end())
		{
			retval = CompactFrameID(0);
		}
		else
			retval = map_it->second;
		return retval;
	}

	CompactFrameID BufferCore::lookupOrInsertFrameNumber(const std::string& frameid_str)
	{
		CompactFrameID retval = 0;
		M_StringToCompactFrameID::iterator map_it = frameIDs_.find(frameid_str);
		if (map_it == frameIDs_.end())
		{
			retval = CompactFrameID(frames_.size());
			frames_.push_back(TimeCacheInterfacePtr());//Just a place holder for iteration
			frameIDs_[frameid_str] = retval;
			frameIDs_reverse.push_back(frameid_str);
		}
		else
			retval = frameIDs_[frameid_str];

		return retval;
	}

	const std::string& BufferCore::lookupFrameString(CompactFrameID frame_id_num) const
	{
		if (frame_id_num >= frameIDs_reverse.size())
		{
			std::stringstream ss;
			ss << "Reverse lookup of frame id " << frame_id_num << " failed!";
			throw LookupException(ss.str());
		}
		else
			return frameIDs_reverse[frame_id_num];
	}

	void BufferCore::createConnectivityErrorString(CompactFrameID source_frame, CompactFrameID target_frame, std::string* out) const
	{
		if (!out)
		{
			return;
		}
		*out = std::string("Could not find a connection between '"+lookupFrameString(target_frame)+"' and '"+
				lookupFrameString(source_frame)+"' because they are not part of the same tree."+
				"Tf has two or more unconnected trees.");
	}

	std::string BufferCore::allFramesAsStringNoLock() const
	{
		std::stringstream mstream;

		TransformStorage temp;

		//  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)

		///regular transforms
		for (unsigned int counter = 1; counter < frames_.size(); counter ++)
		{
			TimeCacheInterfacePtr frame_ptr = getFrame(CompactFrameID(counter));
			if (frame_ptr == NULL)
				continue;
			CompactFrameID frame_id_num;
			if(  frame_ptr->getData(ros::Time(), temp))
				frame_id_num = temp.frame_id_;
			else
			{
				frame_id_num = 0;
			}
			mstream << "Frame "<< frameIDs_reverse[counter] << " exists with parent " << frameIDs_reverse[frame_id_num] << "." <<std::endl;
		}

		return mstream.str();
	}

	struct TimeAndFrameIDFrameComparator
	{
		TimeAndFrameIDFrameComparator(CompactFrameID id)
			: id(id)
		{}

		bool operator()(const P_TimeAndFrameID& rhs) const
		{
			return rhs.second == id;
		}

		CompactFrameID id;
	};

	int BufferCore::getLatestCommonTime(CompactFrameID target_id, CompactFrameID source_id, ros::Time & time, std::string * error_string) const
	{
		// Error if one of the frames don't exist.
		if (source_id == 0 || target_id == 0) return tf2_msgs::TF2Error::LOOKUP_ERROR;

		if (source_id == target_id)
		{
			TimeCacheInterfacePtr cache = getFrame(source_id);
			//Set time to latest timestamp of frameid in case of target and source frame id are the same
			if (cache)
				time = cache->getLatestTimestamp();
			else
				time = ros::Time();
			return tf2_msgs::TF2Error::NO_ERROR;
		}

		std::vector<P_TimeAndFrameID> lct_cache;

		// Walk the tree to its root from the source frame, accumulating the list of parent/time as well as the latest time
		// in the target is a direct parent
		CompactFrameID frame = source_id;
		P_TimeAndFrameID temp;
		uint32_t depth = 0;
		ros::Time common_time = ros::TIME_MAX;
		while (frame != 0)
		{
			TimeCacheInterfacePtr cache = getFrame(frame);

			if (!cache)
			{
				// There will be no cache for the very root of the tree
				break;
			}

			P_TimeAndFrameID latest = cache->getLatestTimeAndParent();

			if (latest.second == 0)
			{
				// Just break out here... there may still be a path from source -> target
				break;
			}

			if (!latest.first.isZero())
			{
				common_time = std::min(latest.first, common_time);
			}

			lct_cache.push_back(latest);

			frame = latest.second;

			// Early out... target frame is a direct parent of the source frame
			if (frame == target_id)
			{
				time = common_time;
				if (time == ros::TIME_MAX)
				{
					time = ros::Time();
				}
				return tf2_msgs::TF2Error::NO_ERROR;
			}

			++depth;
			if (depth > MAX_GRAPH_DEPTH)
			{
				if (error_string)
				{
					std::stringstream ss;
					ss<<"The tf tree is invalid because it contains a loop." << std::endl
						<< allFramesAsStringNoLock() << std::endl;
					*error_string = ss.str();
				}
				return tf2_msgs::TF2Error::LOOKUP_ERROR;
			}
		}

		// Now walk to the top parent from the target frame, accumulating the latest time and looking for a common parent
		frame = target_id;
		depth = 0;
		common_time = ros::TIME_MAX;
		CompactFrameID common_parent = 0;
		while (true)
		{
			TimeCacheInterfacePtr cache = getFrame(frame);

			if (!cache)
			{
				break;
			}

			P_TimeAndFrameID latest = cache->getLatestTimeAndParent();

			if (latest.second == 0)
			{
				break;
			}

			if (!latest.first.isZero())
			{
				common_time = std::min(latest.first, common_time);
			}

			std::vector<P_TimeAndFrameID>::iterator it = std::find_if(lct_cache.begin(), lct_cache.end(), TimeAndFrameIDFrameComparator(latest.second));
			if (it != lct_cache.end()) // found a common parent
			{
				common_parent = it->second;
				break;
			}

			frame = latest.second;

			// Early out... source frame is a direct parent of the target frame
			if (frame == source_id)
			{
				time = common_time;
				if (time == ros::TIME_MAX)
				{
					time = ros::Time();
				}
				return tf2_msgs::TF2Error::NO_ERROR;
			}

			++depth;
			if (depth > MAX_GRAPH_DEPTH)
			{
				if (error_string)
				{
					std::stringstream ss;
					ss<<"The tf tree is invalid because it contains a loop." << std::endl
						<< allFramesAsStringNoLock() << std::endl;
					*error_string = ss.str();
				}
				return tf2_msgs::TF2Error::LOOKUP_ERROR;
			}
		}

		if (common_parent == 0)
		{
			createConnectivityErrorString(source_id, target_id, error_string);
			return tf2_msgs::TF2Error::CONNECTIVITY_ERROR;
		}

		// Loop through the source -> root list until we hit the common parent
		{
			std::vector<P_TimeAndFrameID>::iterator it = lct_cache.begin();
			std::vector<P_TimeAndFrameID>::iterator end = lct_cache.end();
			for (; it != end; ++it)
			{
				if (!it->first.isZero())
				{
					common_time = std::min(common_time, it->first);
				}

				if (it->second == common_parent)
				{
					break;
				}
			}
		}

		if (common_time == ros::TIME_MAX)
		{
			common_time = ros::Time();
		}

		time = common_time;
		return tf2_msgs::TF2Error::NO_ERROR;
	}

	void BufferCore::testTransformableRequests()
	{
		boost::mutex::scoped_lock lock(transformable_requests_mutex_);
		V_TransformableRequest::iterator it = transformable_requests_.begin();
		for (; it != transformable_requests_.end();)
		{
			TransformableRequest& req = *it;

			// One or both of the frames may not have existed when the request was originally made.
			if (req.target_id == 0)
			{
				req.target_id = lookupFrameNumber(req.target_string);
			}

			if (req.source_id == 0)
			{
				req.source_id = lookupFrameNumber(req.source_string);
			}

			ros::Time latest_time;
			bool do_cb = false;
			TransformableResult result = TransformAvailable;
			// TODO: This is incorrect, but better than nothing.  Really we want the latest time for
			// any of the frames
			getLatestCommonTime(req.target_id, req.source_id, latest_time, 0);
			if (!latest_time.isZero() && req.time + cache_time_ < latest_time)
			{
				do_cb = true;
				result = TransformFailure;
			}
			else if (canTransformInternal(req.target_id, req.source_id, req.time, 0))
			{
				do_cb = true;
				result = TransformAvailable;
			}

			if (do_cb)
			{
				{
					boost::mutex::scoped_lock lock2(transformable_callbacks_mutex_);
					M_TransformableCallback::iterator it = transformable_callbacks_.find(req.cb_handle);
					if (it != transformable_callbacks_.end())
					{
						const TransformableCallback& cb = it->second;
						cb(req.request_handle, lookupFrameString(req.target_id), lookupFrameString(req.source_id), req.time, result);
					}
				}

				if (transformable_requests_.size() > 1)
				{
					transformable_requests_[it - transformable_requests_.begin()] = transformable_requests_.back();
				}

				transformable_requests_.erase(transformable_requests_.end() - 1);
			}
			else
			{
				++it;
			}
		}

		// unlock before allowing possible user callbacks to avoid potential detadlock (#91)
		lock.unlock();

		// Backwards compatability callback for tf
		_transforms_changed_();
	}
}
