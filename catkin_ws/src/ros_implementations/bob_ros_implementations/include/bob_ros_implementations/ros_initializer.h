#ifndef _BOB_ROS_IMPLEMENTATIONS_ROS_INITIALIZER_H_
#define _BOB_ROS_IMPLEMENTATIONS_ROS_INITIALIZER_H_

#include <string>

namespace bob
{

	//! \brief Calls ros::init(). This class can be used on its own or as a base class to a ROSSystemHandle.
	//! It is used to guarantee that ros::init is called before any of the member variables in 
	//! ROSSystemHandle are initialized. This works because base class constructors are executed
	//! before derived class members are initialized.
	//! It will also setup ROS visualization and systemUtilities.
	class ROSInitializer
	{
	
		public:
	
			//! \brief Calls ros::init(argc, argv, name) with the provided arguments
			ROSInitializer(int& argc, char** argv, const std::string& name);

			//! \brief Calls ros::init(argc, argv, name) with dummy arguments for argc and argv
			//! \param name The name of the node
			ROSInitializer(const std::string& name);

	};

}

#endif
