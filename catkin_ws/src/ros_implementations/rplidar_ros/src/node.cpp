#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

using namespace rp::standalone::rplidar;

RPlidarDriver * drv = NULL;

void publish_scan(ros::Publisher *pub, 
		rplidar_response_measurement_node_t *nodes, 
		size_t node_count, ros::Time start,
		float scan_time, 
		float angle_min, float angle_max, 
		std::string frame_id)
{
	static int scan_count = 0;
	sensor_msgs::LaserScan scan_msg;

	scan_msg.header.stamp = start;
	scan_msg.header.frame_id = frame_id;
	scan_count++;

	scan_msg.angle_min =  angle_min - M_PI; // angle_min = 0
	scan_msg.angle_max =  angle_max - M_PI; // angle_max = 2pi
	scan_msg.angle_increment = 
		(scan_msg.angle_max - scan_msg.angle_min) / (float)(node_count-1);

	scan_msg.scan_time = scan_time;
	scan_msg.time_increment = scan_time / (float)(node_count-1);

	scan_msg.range_min = 0.15;
	scan_msg.range_max = 6.;

	scan_msg.intensities.resize(node_count);
	scan_msg.ranges.resize(node_count);

	for (size_t i = 0; i < node_count; i++) 
	{
            float read_value = (float) nodes[i].distance_q2/4.0f/1000;
            if (read_value == 0.0)
                scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
            else
                scan_msg.ranges[i] = read_value;
            scan_msg.intensities[i] = (float) (nodes[i].sync_quality >> 2);
        }
	pub->publish(scan_msg);
}

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
	u_result     op_result;
	rplidar_response_device_health_t healthinfo;

	op_result = drv->getHealth(healthinfo);
	if (IS_OK(op_result)) 
	{ 
		printf("RPLidar health status : %d\n", healthinfo.status);

		if (healthinfo.status == RPLIDAR_STATUS_ERROR) 
		{
			fprintf(stderr, "Error, rplidar internal error detected."
					"Please reboot the device to retry.\n");
			return false;
		} 
		else 
		{
			return true;
		}

	} 
	else 
	{
		fprintf(stderr, "Error, cannot retrieve rplidar health code: %x\n", 
				op_result);
		return false;
	}
}

bool stop_motor(std_srvs::Empty::Request &req,
		std_srvs::Empty::Response &res)
{
	if(!drv)
		return false;

	ROS_DEBUG("Stop motor");
	drv->stop();
	drv->stopMotor();
	return true;
}

bool start_motor(std_srvs::Empty::Request &req,
		std_srvs::Empty::Response &res)
{
	if(!drv)
		return false;
	ROS_DEBUG("Start motor");
	drv->startMotor();
	drv->startScan();;
	return true;
}

void initLaser()
{

}

int main(int argc, char * argv[]) 
{
	ros::init(argc, argv, "rplidar_node");

	std::string serial_port;
	int serial_baudrate = 115200;
	std::string frame_id;
	bool angle_compensate = true;

	ros::NodeHandle nh;
	ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
	ros::NodeHandle nh_private("~");
	nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0"); 
	nh_private.param<int>("serial_baudrate", serial_baudrate, 115200); 
	nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
	nh_private.param<bool>("angle_compensate", angle_compensate, "true");

	u_result     op_result;

	// create the driver instance
	drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

	if (!drv) 
	{
		ROS_ERROR("Creating driver failed");
		return -2;
	}

	// make connection...
	if (IS_FAIL(drv->connect(serial_port.c_str(), (_u32)serial_baudrate))) 
	{
		ROS_ERROR("Cannot bind to RPlidar serial port");
		RPlidarDriver::DisposeDriver(drv);
		return -1;
	}

	// check health...
	if (!checkRPLIDARHealth(drv)) 
	{
		RPlidarDriver::DisposeDriver(drv);
		return -1;
	}


	ros::ServiceServer stop_motor_service = nh.advertiseService("stop_motor", stop_motor);
	ros::ServiceServer start_motor_service = nh.advertiseService("start_motor", start_motor);

	// start scan...
	drv->startScan();

	ros::Time timeout_time = ros::Time::now();
	ros::Time start_scan_time;
	ros::Time end_scan_time;
	float scan_duration;
	while (ros::ok()) 
	{

		rplidar_response_measurement_node_t nodes[360*2];
		size_t   count = _countof(nodes);

		start_scan_time = ros::Time::now();

		op_result = drv->grabScanData(nodes, count);
		end_scan_time = ros::Time::now();
		scan_duration = (end_scan_time - start_scan_time).toSec() * 1e-3;

		if (op_result == RESULT_OK) 
		{
			op_result = drv->ascendScanData(nodes, count);

			float angle_min = DEG2RAD(0.0f);
			float angle_max = DEG2RAD(359.0f);
			if (op_result == RESULT_OK) 
			{
				if (angle_compensate) 
				{
					const int angle_compensate_nodes_count = 360;
					const int angle_compensate_multiple = 1;
					int angle_compensate_offset = 0;
					rplidar_response_measurement_node_t angle_compensate_nodes[angle_compensate_nodes_count];

					memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(rplidar_response_measurement_node_t));
					int i = 0, j = 0;

					for( ; i < count; i++ ) 
					{
						if (nodes[i].distance_q2 != 0) 
						{
							float angle = (float)((nodes[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
							int angle_value = (int)(angle * angle_compensate_multiple);
							if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
							for (j = 0; j < angle_compensate_multiple; j++) 
							{
								angle_compensate_nodes[angle_value-angle_compensate_offset+j] = nodes[i];
							}
						}
					}

					publish_scan(&scan_pub, angle_compensate_nodes, angle_compensate_nodes_count,
							start_scan_time, scan_duration,  
							angle_min, angle_max, 
							frame_id);
				} 
				else 
				{
					int start_node = 0, end_node = 0;
					int i = 0;
					// find the first valid node and last valid node
		
					while (nodes[i++].distance_q2 == 0);
					start_node = i-1;
					i = count -1;
					while (nodes[i--].distance_q2 == 0);
					end_node = i+1;

					angle_min = DEG2RAD((float)(nodes[start_node].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
					angle_max = DEG2RAD((float)(nodes[end_node].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);

					publish_scan(&scan_pub, &nodes[start_node], end_node-start_node +1, 
							start_scan_time, scan_duration,  
							angle_min, angle_max, 
							frame_id);
				}
			} 
			else if (op_result == RESULT_OPERATION_FAIL) 
			{
				// All the data is invalid, just publish them
				float angle_min = DEG2RAD(0.0f);
				float angle_max = DEG2RAD(359.0f);

				publish_scan(&scan_pub, nodes, count, 
						start_scan_time, scan_duration,  
						angle_min, angle_max, 
						frame_id);
			}
			timeout_time = ros::Time::now();
		}
		else if (ros::Time::now() - timeout_time > ros::Duration(3))
		{
			drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
			timeout_time = ros::Time::now();
			drv->startScan();
			ROS_INFO("Resetting!");
		}

		ros::spinOnce();
	}

	// done!
	RPlidarDriver::DisposeDriver(drv);
	return 0;
}
