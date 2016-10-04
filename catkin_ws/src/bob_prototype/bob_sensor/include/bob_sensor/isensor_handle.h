#ifndef _BOB_SENSOR_ISENSOR_HANDLE_H_
#define _BOB_SENSOR_ISENSOR_HANDLE_H_

namespace bob
{

	class ITransformHandle;
	class IProximitySensor;
	class IOdometryHandle;
	class IBumperHandle;
	class IScanSensorHandle;
	class ITransformNotifier;

	//! \brief Interface for obtaining sensor information from the robot.
	//! Provides access to sensor handles, which provide generalized sensor information.
	class ISensorHandle
	{

		public:

			//! \brief Handle to obtain information about the robot's position and orientation.
			//! \return A reference to an ITransformHandle
			virtual const ITransformHandle& getTransformHandle() const = 0;

			//! \brief Handle to sensor information that gives information about the environment
			//! around the robot. 
			//! \return A reference to an IProximitySensor
			virtual const IProximitySensor& getProximitySensor() const = 0;

			//! \brief Handle used to obtain odometry information, such as speed of robot movement.
			//! \return A reference to an IOdometryHandle
			virtual const IOdometryHandle& getOdometryHandle() const = 0;

			//! \brief Handle to get information about current bumper state.
			//! \return A reference to an IBumperHandle
			virtual const IBumperHandle& getBumperHandle() const = 0;

			virtual const IScanSensorHandle& getScanSensorHandle() const = 0;

			//! \brief A non-const version of this function is required because of the design of the
			//! bumper system. Currently the bumper system uses a sort of polling instead of interrupts.
			//! There needs to be a facility to clear the bumper flag to prevent missed events, which
			//! implies non-constness. This non-const access is limited to SimpleCommander. When
			//! the system is moved to embedded this design will need to change to interrupts.
			//! \return A reference to an IBumperHandle
			virtual IBumperHandle& getBumperHandle() = 0;

			virtual ITransformNotifier& getTransformNotifier() = 0;

	};

}

#endif
