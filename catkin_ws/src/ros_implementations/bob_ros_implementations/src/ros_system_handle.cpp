#include <bob_ros_implementations/ros_system_handle.h>

#include <bob_grid_map/toolbox.h>

namespace bob
{

	ROSSystemHandle::ROSSystemHandle(int& argc, char** argv, const std::string& name) : 
		ROSInitializer(argc, argv, name),
		sensorHandle(),
		mapSender(lockableMap),
		algorithm(sensorHandle, mapSender),
		slamThread(boost::ref(algorithm)),
		controlHandle(sensorHandle, lockableMap, velocityPublisher)
	{
		waitForMapAvailable(lockableMap);
	}

	ISensorHandle& ROSSystemHandle::getSensorHandle()
	{
		return sensorHandle;
	}

	LockableMap& ROSSystemHandle::getLockableMap()
	{
		return lockableMap;
	}

	ControlHandle& ROSSystemHandle::getControlHandle()
	{
		return controlHandle;
	}

	IVelocityPublisher& ROSSystemHandle::getVelocityPublisher()
	{
		return velocityPublisher;
	}

	ROSSystemHandle::~ROSSystemHandle()
	{
		slamThread.join();
	}

}
