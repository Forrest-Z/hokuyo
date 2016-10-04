#ifndef _BOB_CONTROL_RECOVERY_BEHAVIOR_H_
#define _BOB_CONTROL_RECOVERY_BEHAVIOR_H_

namespace bob
{

	class SimpleCommander;
	class ISensorHandle;
	class ITransformHandle;

	void recoverFromBumperEvent(SimpleCommander& simpleCommander, const ISensorHandle& sensorHandle);

	bool canBackUp(const ISensorHandle& sensorHandle);

}

#endif
