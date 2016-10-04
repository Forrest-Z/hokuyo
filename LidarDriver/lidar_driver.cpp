/*
 * LidarDriver.cpp
 *
 *  Created on: Aug 29, 2016
 *      Author: peter
 *
 *      To run, make sure you're on root.  So use sudo -s first
 */

/////////////////////////////////////////////////
// Serial port interface program               //
/////////////////////////////////////////////////
#include "robotleo_lidar_driver.hpp"
#include <stdio.h>
#include <vector>

#include <iostream>

#include <fstream>

int main(void)
{
	char *portName = "/dev/ttyUSB0";
	std::ofstream myfile1;
	myfile1.open ("Measurement by RoboLeo_60cm_1.txt");
	myfile1 << "Measurement by RoboLeo.\n";

	int count=1;
	for(int n=1;n<=40;++n){
	

	RobotLeoLidarDriver myLidar(portName);

	std::vector<int> result;
	int total=0;
	if (myLidar.isReady())
	{
		result = myLidar.getDistances();
		//myfile1 << "Number of the measurement: " << count << std::endl;
		printf("Number of the measurement: %d\n", count);
	}
	else
		printf("lidar isn't ready!\n");

	for (int i = 114; i <= 114; i++){ //results from left to right... or counter-clockwise 
//changed by hao yuan. original code is ( int i = 0; i < result.size(); i++ ) 
//total is the total number of the test distance 54 to 56 (increment .5)
		myfile1 <<  result[i] << std::endl;
		printf("degree: %.1f, distance: %d\n", i * 0.5-56, result[i]);
	

}	
	count++;
	//printf("average distance (mm): %d\n", total/5);
}
	myfile1.close();
	std::cout<<"data collection completed"<<std::endl;
	return (0);

} //main
