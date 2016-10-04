#ifndef _BOB_TEST_ROS_FLEXIBLE_SYSTEM_HANDLE_H_
#define _BOB_TEST_ROS_FLEXIBLE_SYSTEM_HANDLE_H_

#include <boost/thread.hpp>

#include <bob_ros_implementations/ros_map_sender.h>

#include <bob_slam/slam_ros.h>

#include <bob_sensor/isensor_handle.h>
#include <bob_grid_map/toolbox.h>

#include <bob_ros_implementations/ros_flexible_sensor_handle.h>
#include <bob_ros_implementations/ros_sensor_handle.h>
#include <bob_control_handle/control_handle.h>
#include <bob_ros_implementations/ros_velocity_publisher.h>
#include <bob_ros_implementations/ros_spinner.h>
#include <bob_ros_implementations/ros_initializer.h>

#include <bob_system/isystem_handle.h>

#include <bob_toolbox/logging.h>

namespace bob
{

	class ROSFlexibleSystemHandle : private ROSInitializer, public ISystemHandle
	{

		public:

			//! \brief Create an instance. Start the mapping system and
			//! initialize sensors.
			//! \param argc argc value from main(argc, argv) function
			//! \param argv argv value from main(argc, argv) function
			//! \param name The desired name of the node
			ROSFlexibleSystemHandle(std::string name = "bob_system") :
			ROSInitializer(name),
			sensorHandle(),
			mapSender(lockableMap),
			algorithm(new SlamROS(sensorHandle, mapSender)),
			controlHandle(new ControlHandle(sensorHandle, lockableMap, velocityPublisher))
			{}
	
			void resetScanSensorHandle(std::unique_ptr<IScanSensorHandle> newScanSensorHandle)
			{
				sensorHandle.resetScanSensor(std::move(newScanSensorHandle));
			}
		
			virtual ISensorHandle& getSensorHandle()
			{
				return sensorHandle;
			}

			virtual LockableMap& getLockableMap()
			{
				return lockableMap;
			}
	
			virtual ControlHandle& getControlHandle()
			{
				return *controlHandle;
			}

			virtual IVelocityPublisher& getVelocityPublisher()
			{
				return velocityPublisher;
			}

			~ROSFlexibleSystemHandle()
			{
				slamThread.join();
			}

			void start()
			{
				slamThread = boost::thread(boost::ref(*algorithm));
				waitForMapAvailable(lockableMap);
			}

		private:

			ROSSpinner spinner;

			ROSVelocityPublisher velocityPublisher;

			ROSFlexibleSensorHandle sensorHandle;

			LockableMap lockableMap;

			ROSMapSender mapSender;

			std::unique_ptr<SlamROS> algorithm;

			boost::thread slamThread; 

			std::unique_ptr<ControlHandle> controlHandle;

	};

}

#endif
