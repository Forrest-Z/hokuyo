#ifndef _BOB_LIDAR_PASSABLE_RANGE_H_
#define _BOB_LIDAR_PASSABLE_RANGE_H_

#include <bob_toolbox/angular_range.h>
#include <bob_sensor/iscan_sensor_handle.h>


namespace bob
{

	//! This class is used to calculate a best angular range for the robot to pass beside.
	class PassableRange
	{
		public:
			
			PassableRange(IScanSensorHandle& lidarHandle, float maxCheckRange, float margin): 
				lidarHandle(lidarHandle), 
				marginDist(margin),
				maxCheckRange(maxCheckRange)
			 {}
		
			AngularRange getPassableRange();
			
		private:
			
			IScanSensorHandle& lidarHandle;

			float maxCheckRange;

			float marginDist;
			

	};

}

#endif
