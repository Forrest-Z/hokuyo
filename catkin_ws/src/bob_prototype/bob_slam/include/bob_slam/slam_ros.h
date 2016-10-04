#ifndef _BOB_SLAM_SLAM_ROS_H_
#define _BOB_SLAM_SLAM_ROS_H_

#include <bob_sensor/itransform_handle.h>

#include <bob_slam/slam_processor.h>
#include <bob_slam/slam_processor_narrow_fov.h>
#include <bob_grid_map/map_sender.h>
#include <bob_sensor/isensor_handle.h>

#include <memory>

namespace bob
{

	class SlamROS
	{
		public:

			SlamROS(ISensorHandle& sensorHandle, MapSender& mapSender);

			void operator()();

		private:

			SlamProcessorNarrowFOV slamProcessor;

			ISensorHandle& sensorHandle;

			MapSender& mapSender;

	};

}

#endif
