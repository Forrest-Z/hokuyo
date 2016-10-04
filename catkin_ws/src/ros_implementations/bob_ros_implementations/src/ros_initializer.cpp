#include <bob_ros_implementations/ros_initializer.h>

#include <bob_ros_implementations/ros_system_utilities.h>
#include <bob_ros_implementations/ros_visualizer.h>

#include <cstring>

namespace bob
{

	ROSInitializer::ROSInitializer(int& argc, char** argv, const std::string& name)
	{
		ros::init(argc, argv, name);
		configureROSSystemUtilities();
		configureROSVisualization();
	}

	ROSInitializer::ROSInitializer(const std::string& name)
	{
		// Set up the dummy params
		char cName[100];
		strcpy(&cName[0], name.c_str());
		char * argv[] = { &cName[0], NULL };
		int argc = 1;

		ros::init(argc, argv, name);
		configureROSSystemUtilities();
		configureROSVisualization();
	}

}

