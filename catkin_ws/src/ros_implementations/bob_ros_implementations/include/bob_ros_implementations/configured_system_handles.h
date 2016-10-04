#ifndef _BOB_ROS_IMPLEMENTATIONS_CONFIGURED_SYSTEM_HANDLES_H_
#define _BOB_ROS_IMPLEMENTATIONS_CONFIGURED_SYSTEM_HANDLES_H_

#include <memory>

#include <bob_system/isystem_handle.h>

#include <bob_toolbox/angular_range.h>

namespace bob
{

	std::unique_ptr<ISystemHandle> limitedLidarSystemHandle(SimpleAngularRange limit);

}

#endif
