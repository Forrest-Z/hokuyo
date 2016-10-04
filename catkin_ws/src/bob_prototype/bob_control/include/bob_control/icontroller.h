#ifndef _BOB_CONTROL_ICONTROLLER_H_
#define _BOB_CONTROL_ICONTROLLER_H_

#include <boost/shared_ptr.hpp>

namespace bob
{

	class ISensorHandle;
	class Velocity2D;

	class IController
	{

		public:

			typedef boost::shared_ptr<IController> shared_ptr;

			virtual Velocity2D nextCommand(const ISensorHandle& sensorHandle) = 0;
	
			virtual float desiredFrequency()
			{
				return 15.0;
			}


			virtual ~IController()
			{}
	};

}

#endif
