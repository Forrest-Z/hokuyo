#ifndef _BOB_SENSOR_LIDAR_SCAN_H_
#define _BOB_SENSOR_LIDAR_SCAN_H_

#include <vector>

#include <iterator>

#include <bob_sensor/lidar_beam.h>
#include <bob_sensor/lidar_scan_iterator.h>

namespace bob
{

	//! \brief Class which represents a client-friendly Lidar scan.
	//! The data in the object is only accessible via iterator.
	class LidarScan
	{

		public:

			//! \brief Initialize with empty data
			LidarScan() {};

			//! \brief Constructors used to create the scan from segment(s)
			LidarScan(std::vector<LidarScanSegment> segments) :
			segments(segments) {}

			LidarScan(LidarScanSegment segment) :
			segments(1, segment) {}

			typedef LidarIterator iterator;

			inline const iterator begin() const
			{
				return iterator(segments);
			}

			//! Used to determine when to stop iterating
			inline const iterator end() const
			{
				return iterator((segments.end() - 1)->end(), segments.end(), segments.end());
			}

			inline const int size() const
			{
				int val = 0;
				for (std::vector<LidarScanSegment>::const_iterator segmentItr = segments.begin();
				segmentItr != segments.end();
				++segmentItr)
				{
					val += segmentItr->size();
				}
				return val;
			}

		private:

			//! The lidar segments that make up the scan
			std::vector<LidarScanSegment> segments;

	};

}

#endif
