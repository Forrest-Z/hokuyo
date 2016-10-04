#ifndef _BOB_COVERAGE_COVERAGE_TRACKER_H_
#define _BOB_COVERAGE_COVERAGE_TRACKER_H_

#include <bob_sensor/itransform_handle.h>

#include <bob_coverage/discrete_area.h>
#include <bob_toolbox/internally_locked.h>
#include <bob_toolbox/periodic_thread.h>
#include <bob_system/periodic_threader.h>

#include <bob_config/config.h>

namespace bob
{

	class CoverageTrackerThread : public PeriodicThread
	{

		public:

			CoverageTrackerThread(const ITransformHandle& transformHandle) : 
				transformHandle(transformHandle),
				coveredArea(Config::MAP_RESOLUTION)
		{}			

			//! This will continuously update the robot area data
			//! with the robot's position
			virtual void operator()();

			DiscreteArea getCoveredArea() const;

		private:

			const ITransformHandle& transformHandle;

			//! The coverage area of the robot
			DiscreteArea coveredArea;

			//! Inserts a new discrete area into the mask,
			//! with proper mutex for threading safety.
			void insertMask(const DiscreteArea& mask);

			mutable boost::mutex coveredPointsMutex;


	};

	//! \brief Class which tracks the robot's position. Periodically polls the robot position
	//! and updates a stored DiscreteArea with the circular area under the robot's current position.
	//! This class runs its own thread to track the area.
	class CoverageTracker
	{

		public: 

			//! \brief Construct a coverage tracker object.
			//! \param transformHandle Provides transform information, giving the robot location
			//! \param period The period at which to run the thread.
			CoverageTracker(const ITransformHandle& transformHandle, float period) :
				thread(transformHandle),
				threader(thread, period)
		{}

			//! \brief Get the area where the robot has travelled
			//! \return The area where the robot has covered
			DiscreteArea getCoveredArea() const; 


			//! \brief Starts recording the robot's position.
			inline void start()
			{
				threader.start();
			}

		private:

			//! The thread object that does the work.
			CoverageTrackerThread thread;

			//! The "threader" which runs the thread.
			PeriodicThreader threader;

	};

}

#endif
