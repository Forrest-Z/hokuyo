#ifndef _BOB_SENSOR_LIDAR_SCAN_ITERATOR_H_
#define _BOB_SENSOR_LIDAR_SCAN_ITERATOR_H_

#include <bob_sensor/lidar_beam.h>

namespace bob
{

	//! \brief Defines an iterator which iterates over a LidarSegment object
	//! It contains all the distances for a segment of a lidar scan, as well
	//! as the data required to calculate the angles. The angles are
	//! ultimately calculated during iteration.
	class LidarSegmentIterator
	{

		//! Required so that LidarScanSegment can access the private constructor
		friend class LidarScanSegment;
		
		public:

			inline LidarSegmentIterator& operator++()
			{
				//! Update the distance iterator and angle
				++currentDistance;
				currentAngle += increment;;
			}

			inline const LidarBeam& operator*()
			{
				//! beamToSend is updated here because we need to check end()
				//! condition before dereferencing currentDistance
				beamToSend = LidarBeam(currentAngle, *currentDistance);
				return beamToSend;
			}
			
			inline bool operator==(const LidarSegmentIterator& rhs)
			{
				return (currentDistance == rhs.currentDistance);
			}

			inline bool operator!=(const LidarSegmentIterator& rhs)
			{
				return !operator==(rhs);
			}			
	
			inline const LidarBeam* operator->()
			{
				beamToSend = LidarBeam(currentAngle, *currentDistance);
				return &beamToSend;
			}

		private:

			//! The constructor is private because no one should be creating objects of this type except its friend, LidarScanSegment
			LidarSegmentIterator(float currentAngle, float increment, std::vector<float>::const_iterator currentDistance) :
				currentAngle(currentAngle),
				increment(increment),
				currentDistance(currentDistance) {}

			LidarBeam beamToSend;
			float currentAngle;
			float increment;
			std::vector<float>::const_iterator currentDistance;

	};

	//! Defines a section of a lidar scan.
	class LidarScanSegment
	{
		public:

			LidarScanSegment(float startingAngle, float increment, std::vector<float> distances) : 
				startingAngle(startingAngle),
				increment(increment),
				distances(distances)
				{}

			typedef LidarSegmentIterator iterator;

			//! Obtain iterator to start of segment 
			inline const iterator begin() const
			{
				return iterator(startingAngle, increment, distances.begin());
			}	

			//! Obtain first-past-end iterator (for for-loop comparison)
			inline const iterator end() const
			{
				return iterator(startingAngle + increment * distances.size(), increment, distances.end());
			}	

			inline int size() const
			{
				return distances.size();
			}

		private:

			//! The for the beam at distances[0]
			float startingAngle;

			//! The difference in angle between successive beams
			float increment;
	
			//! The beam distance data
			std::vector<float> distances;
	};

	//! This iterator is used to iterate over LidarScan objects
	//! It allows clients to iterate over multiple LidarScanSegment seemlessly
	//! as if they were a single iterator.
	//! You can use the iterator similar to an STL iterator.
	class LidarIterator
	{

		//! Required so that LidarScan can access the private constructor
		friend class LidarScan;	

		public:

			inline LidarIterator& operator++()
			{
				++positionItr;
				if (positionItr == segmentItr->end()) 
				{
					//! Go on to the next segment
					++segmentItr;

					//! Make sure not to dereference segmentItr if it is end()
					if (segmentItr != segmentEnd)
						positionItr = segmentItr->begin();
				}
			}

			inline const LidarBeam& operator*()
			{
				return *positionItr;
			}
			
			inline bool operator==(const LidarIterator& rhs)
			{
				return (positionItr == rhs.positionItr && segmentItr == rhs.segmentItr);
			}

			inline bool operator!=(const LidarIterator& rhs)
			{
				return !operator==(rhs);
			}			
	
			inline const LidarBeam* operator->()
			{
				return &(*positionItr);
			}

		private:

			//! Private constructors meant to only be used by LidarScan
			LidarIterator(const std::vector<LidarScanSegment>& toIterate) :
				positionItr(toIterate.begin()->begin()),
				segmentItr(toIterate.begin()),
				segmentEnd(toIterate.end()) {}	

			LidarIterator(LidarScanSegment::iterator positionItr, std::vector<LidarScanSegment>::const_iterator segmentItr, std::vector<LidarScanSegment>::const_iterator segmentEnd) :
				positionItr(positionItr),
				segmentItr(segmentItr),
				segmentEnd(segmentEnd) {}	

			//! Iterator that points to the current beam
			LidarScanSegment::iterator positionItr;
		
			//! Iterator that points to the current LidarScanSegment
			std::vector<LidarScanSegment>::const_iterator segmentItr;
	
			//! Iterator used for end condition
			std::vector<LidarScanSegment>::const_iterator segmentEnd;
	
	};

}

#endif
