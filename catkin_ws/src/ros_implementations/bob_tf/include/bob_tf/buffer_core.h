#ifndef _BOB_TF_BUFFER_CORE_H_
#define _BOB_TF_BUFFER_CORE_H_

#include "transform_storage.h"

#include <boost/signals2.hpp>

#include <string>

#include "ros/duration.h"
#include "ros/time.h"
#include "geometry_msgs/TransformStamped.h"

#include <boost/unordered_map.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

namespace bob
{

	typedef std::pair<ros::Time, CompactFrameID> P_TimeAndFrameID;
	typedef uint32_t TransformableCallbackHandle;
	typedef uint64_t TransformableRequestHandle;

	class TimeCacheInterface;
	typedef boost::shared_ptr<TimeCacheInterface> TimeCacheInterfacePtr;

	enum TransformableResult
	{
		TransformAvailable,
		TransformFailure,
	};

	class BufferCore
	{
		public:
			/************* Constants ***********************/
			static const int DEFAULT_CACHE_TIME = 10;  //!< The default amount of time to cache data in seconds
			static const uint32_t MAX_GRAPH_DEPTH = 1000UL;  //!< Maximum graph search depth (deeper graphs will be assumed to have loops)

			BufferCore(ros::Duration cache_time_ = ros::Duration(DEFAULT_CACHE_TIME));

			/** \brief Clear all data */
			void clear();

			bool setTransform(const geometry_msgs::TransformStamped& transform, const std::string & authority, bool is_static = false);

			geometry_msgs::TransformStamped 
				lookupTransform(const std::string& target_frame, const std::string& source_frame,
						const ros::Time& time) const;

			geometry_msgs::TransformStamped
				lookupTransform(const std::string& target_frame, const ros::Time& target_time,
						const std::string& source_frame, const ros::Time& source_time,
						const std::string& fixed_frame) const;

			bool canTransform(const std::string& target_frame, const std::string& source_frame,
					const ros::Time& time, std::string* error_msg = NULL) const;

			bool canTransform(const std::string& target_frame, const ros::Time& target_time,
					const std::string& source_frame, const ros::Time& source_time,
					const std::string& fixed_frame, std::string* error_msg = NULL) const;

			typedef boost::function<void(TransformableRequestHandle request_handle, const std::string& target_frame, const std::string& source_frame,
					ros::Time time, TransformableResult result)> TransformableCallback;


			CompactFrameID _lookupFrameNumber(const std::string& frameid_str) const { 
				return lookupFrameNumber(frameid_str); 
			}
			CompactFrameID _lookupOrInsertFrameNumber(const std::string& frameid_str) {
				return lookupOrInsertFrameNumber(frameid_str); 
			}


			CompactFrameID _validateFrameId(const char* function_name_arg, const std::string& frame_id) const {
				return validateFrameId(function_name_arg, frame_id);
			}

		private:

			std::string allFramesAsStringNoLock() const;  

			typedef std::vector<TimeCacheInterfacePtr> V_TimeCacheInterface;
			V_TimeCacheInterface frames_;

			/** \brief A mutex to protect testing and allocating new frames on the above vector. */
			mutable boost::mutex frame_mutex_;

			/** \brief A map from string frame ids to CompactFrameID */
			typedef boost::unordered_map<std::string, CompactFrameID> M_StringToCompactFrameID;
			M_StringToCompactFrameID frameIDs_;
			/** \brief A map from CompactFrameID frame_id_numbers to string for debugging and output */
			std::vector<std::string> frameIDs_reverse;
			/** \brief A map to lookup the most recent authority for a given frame */
			std::map<CompactFrameID, std::string> frame_authority_;


			///! How long to cache transform history
			ros::Duration cache_time_;

			typedef boost::unordered_map<TransformableCallbackHandle, TransformableCallback> M_TransformableCallback;
			M_TransformableCallback transformable_callbacks_;
			uint32_t transformable_callbacks_counter_;
			boost::mutex transformable_callbacks_mutex_;

			struct TransformableRequest
			{
				ros::Time time;
				TransformableRequestHandle request_handle;
				TransformableCallbackHandle cb_handle;
				CompactFrameID target_id;
				CompactFrameID source_id;
				std::string target_string;
				std::string source_string;
			};
			typedef std::vector<TransformableRequest> V_TransformableRequest;
			V_TransformableRequest transformable_requests_;
			boost::mutex transformable_requests_mutex_;
			uint64_t transformable_requests_counter_;

			//! Backwards compatability for tf message_filter
			typedef boost::signals2::signal<void(void)> TransformsChangedSignal;
			///! Signal which is fired whenever new transform data has arrived, from the thread the data arrived in
			TransformsChangedSignal _transforms_changed_;

			TimeCacheInterfacePtr getFrame(CompactFrameID c_frame_id) const;

			TimeCacheInterfacePtr allocateFrame(CompactFrameID cfid, bool is_static);

			bool warnFrameId(const char* function_name_arg, const std::string& frame_id) const;
			CompactFrameID validateFrameId(const char* function_name_arg, const std::string& frame_id) const;

			///! String to number for frame lookup with dynamic allocation of new frames
			CompactFrameID lookupFrameNumber(const std::string& frameid_str) const;

			///! String to number for frame lookup with dynamic allocation of new frames
			CompactFrameID lookupOrInsertFrameNumber(const std::string& frameid_str);

			///Number to string frame lookup may throw LookupException if number invalid
			const std::string& lookupFrameString(CompactFrameID frame_id_num) const;

			void createConnectivityErrorString(CompactFrameID source_frame, CompactFrameID target_frame, std::string* out) const;

			/**@brief Return the latest rostime which is common across the spanning set
			 * zero if fails to cross */
			int getLatestCommonTime(CompactFrameID target_frame, CompactFrameID source_frame, ros::Time& time, std::string* error_string) const;

			template<typename F>
				int walkToTopParent(F& f, ros::Time time, CompactFrameID target_id, CompactFrameID source_id, std::string* error_string) const;

			/**@brief Traverse the transform tree. If frame_chain is not NULL, store the traversed frame tree in vector frame_chain.
			 * */
			template<typename F>
				int walkToTopParent(F& f, ros::Time time, CompactFrameID target_id, CompactFrameID source_id, std::string* error_string, std::vector<CompactFrameID> *frame_chain) const;

			void testTransformableRequests();
			bool canTransformInternal(CompactFrameID target_id, CompactFrameID source_id,
					const ros::Time& time, std::string* error_msg) const;
			bool canTransformNoLock(CompactFrameID target_id, CompactFrameID source_id,
					const ros::Time& time, std::string* error_msg) const;


	};


};

#endif
