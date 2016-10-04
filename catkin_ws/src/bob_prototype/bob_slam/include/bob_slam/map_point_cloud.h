#ifndef _BOB_SLAM_MAP_POINT_CLOUD_H_
#define _BOB_SLAM_MAP_POINT_CLOUD_H_

#include <bob_sensor/lidar_scan.h>
#include <bob_toolbox/generic_point.h>
#include <bob_grid_map/location_mapper.h>
#include <vector>

namespace bob
{

	//! \brief Convert scan message to end points with the map resolution and store them.
	class MapPointCloud
	{

		public:	
	
			typedef std::vector<GenericPoint<float> > storage_type; 
			typedef storage_type::const_iterator iterator;

			//! \brief Constructor. Convert scan message to end points with the map resolution and store them.
			MapPointCloud(const LidarScan& scan, const LocationMapper& locationMapper);

			//! \brief Check if the point cloud is empty
			inline bool isEmpty()
			{
				return points.size() == 0;
			}

			//! \brief An iterator pointing to the begin of the point cloud.
			inline iterator begin() const
			{
				return points.begin();
			}

			
			//! \brief An iterator pointing to the end of the point cloud.
			inline iterator end() const
			{
				return points.end();
			}

		private:

			//! End points
			std::vector<GenericPoint<float> > points;

	};

}

#endif
