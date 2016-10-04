#ifndef _BOB_TEST_ROS_SYSTEM_HANDLE_H_
#define _BOB_TEST_ROS_SYSTEM_HANDLE_H_

#include <boost/thread.hpp>

#include <bob_ros_implementations/ros_map_sender.h>

#include <bob_slam/slam_ros.h>

#include <bob_sensor/isensor_handle.h>

#include <bob_ros_implementations/ros_sensor_handle.h>
#include <bob_control_handle/control_handle.h>
#include <bob_ros_implementations/ros_velocity_publisher.h>
#include <bob_ros_implementations/ros_spinner.h>
#include <bob_ros_implementations/ros_initializer.h>

#include <bob_system/isystem_handle.h>

namespace bob
{

	//! \brief Groups together the core compenents of the ROS system, excluding the actual algorithms
	//! used for exploration and coverage. A map is produced using the SLAM algorithm. Sensors are control
	//! systems are also intialized. The map and the handles can be accessed via the member functions,
	//! which expose references to these objects.
	//! The class will call ros::init(), and contains a ROSSpinner which will constantly update the sensor data
	//! by calling ros::spin() on a separate thread.
	class ROSSystemHandle : private ROSInitializer, public ISystemHandle
	{

		public:

			//! \brief Create an instance. Start the mapping system and
			//! initialize sensors.
			//! \param argc argc value from main(argc, argv) function
			//! \param argv argv value from main(argc, argv) function
			//! \param name The desired name of the node
			ROSSystemHandle(int& argc, char** argv, const std::string& name);

			//! \brief Obtain a reference to the ISensorHandle member
			//! \return A reference to the ISensorHandle member
			virtual ISensorHandle& getSensorHandle();

			//! \brief Obtain a reference to the LockableMap member
			//! \return A reference to the LockableMap member
			virtual LockableMap& getLockableMap();
	
			//! \brief Obtain a reference to the ControlHandle member
			//! \return A reference to the ControlHandle member
			virtual ControlHandle& getControlHandle();

			virtual IVelocityPublisher& getVelocityPublisher();

			//! \brief Destructor will join the mapping thread on exit
			~ROSSystemHandle();

		private:

			//! Updates sensor data from ROS by running ros::spin on a separate thread
			ROSSpinner spinner;

			//! Publishes velocity to a ROS topic
			ROSVelocityPublisher velocityPublisher;

			//! Access to sensor data
			ROSSensorHandle sensorHandle;

			//! Mutex protected map
			LockableMap lockableMap;

			ROSMapSender mapSender;

			//! SLAM algorithm
			SlamROS algorithm;

			//! Thread on which the slam algorithm will run
			boost::thread slamThread; 

			//! Provides access to robot control systems
			ControlHandle controlHandle;

	};

}

#endif
