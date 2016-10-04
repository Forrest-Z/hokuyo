#include <bob_system/system_utilities.h>

#include <bob_toolbox/logging.h>

namespace bob
{

	bool SystemUtilities::ok() const
	{
		LOG_ERROR("Calling uninitialized SystemUtilities object");
		return true;
	}

	void SystemUtilities::sleep(float seconds)
	{
		LOG_ERROR("Calling uninitialized SystemUtilities object");
	}

	RateTimer SystemUtilities::rateTimer(float frequency)
	{
		LOG_ERROR("Calling uninitialized SystemUtilities object");
		return std::unique_ptr<IRateTimer>(nullptr);		
	}

	Timer SystemUtilities::timer()
	{
		LOG_ERROR("Calling uninitialized SystemUtilities object");
		return std::unique_ptr<ITimer>(nullptr);		
	}

	std::unique_ptr<SystemUtilities> systemUtilities(new SystemUtilities());

}

