#include <bob_ros_implementations/ros_initializer.h>
#include <bob_ros_implementations/ros_lidar_handle.h>
#include <bob_ros_implementations/ros_spinner.h>

#include <bob_sensor/lidar_scan.h>

#include <bob_system/system_utilities.h>

#include <bob_toolbox/logging.h>
#include <bob_toolbox/angles.h>

#include <iostream>
#include <string>
#include <fstream>


using namespace bob;

int main(int argc, char** argv)
{
	// Initializes ros
	ROSInitializer(argc, argv, "lidar_test");

	// Spins on separate thread
	ROSSpinner spinner;

	// Create lidar reader
	ROSLidarHandle lidarHandle;
	
	// Specify a range to extract
	SimpleAngularRange range(toRadian(-10), toRadian(10));
	int n=0;
	float display=0;
	std::ofstream myfile;
	
	myfile.open ("Measurement by Hokuyo_60cm.txt");
	myfile << "Measurement by Hokuyo.\n";
	while (systemUtilities->ok())
	{
		//LidarScan mostRecentData = lidarHandle.getLidarData();
		LidarScan mostRecentData = lidarHandle.getLidarData(range); // <-- This should work now
		
		/*
		float sum = 0;
		for (auto lidarIterator = mostRecentData.begin(); lidarIterator != mostRecentData.end(); ++lidarIterator)
		{

		}
		*/
		
		auto lidarIterator = mostRecentData.begin();
		display=lidarIterator->range;
		
		/*
		LOG_TEMP("Read lidar range: " <<lidarIterator->range);
		systemUtilities->sleep(1);
*/
		n+=1;
		
 		
  		
  		//myfile << "integer " << n << " distance: " << display*1000 << std::endl;
		myfile << display*1000 << std::endl;
		if(n==40){
  		myfile.close();
		std::cout<<"data collection completed"<<std::endl;
		break;
		}

		printf("display number: %f\n",display*1000);
		//LOG_TEMP("Read lidar range: " <<lidarIterator->range);
	
		// Sleep
		systemUtilities->sleep(.1);
		
	
	

		
	}
	
	return 0;
}
