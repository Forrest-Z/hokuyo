#include <bob_slam/map_point_cloud.h>

namespace bob
{

	MapPointCloud::MapPointCloud(const LidarScan& scan, const LocationMapper& locationMapper)
	{
		float mapResolution = locationMapper.getResolution();
		for (LidarScan::iterator beamItr = scan.begin(); beamItr != scan.end(); ++beamItr)
		{
			float distance = beamItr->range;
			float angle = beamItr->angle;

			if (distance < Config::MAX_SENSOR_RANGE && !isnan(distance))
			{
				GenericPoint<float> newPoint = (distance / mapResolution) * unitVector<float>(angle);
				points.push_back(newPoint);
			}
		}
	}

}

