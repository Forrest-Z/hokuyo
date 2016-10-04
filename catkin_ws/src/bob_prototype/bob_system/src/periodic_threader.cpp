#include <bob_system/periodic_threader.h>

#include <bob_system/system_utilities.h>

namespace bob
{

	PeriodicThreader::PeriodicThreader(PeriodicThread& threadable, float period) :
		running(false),
		threadable(threadable),
		rateTimer(systemUtilities->rateTimer(1.0/period))
	{
	}

	void PeriodicThreader::start()
	{
		running = true;

		// Start up the thread
		thread = boost::thread(boost::bind(&PeriodicThreader::threadLoop, this));			
	}

	void PeriodicThreader::threadLoop()
	{
		while(running)
		{
			rateTimer->startTimer();

			// Run the function
			threadable();

			// Wait for periodicity
			rateTimer->sleepRemaining();
		}

	}

	PeriodicThreader::~PeriodicThreader()
	{
		if (running)
		{
			// Raise flag to stop thread
			running = false;

			// Join thread to prevent crashing on destruction
			thread.join();
		}
	}
}
