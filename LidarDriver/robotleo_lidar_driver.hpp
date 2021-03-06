#include <vector>

class RobotLeoLidarDriver
{ // a.k.a. 30$ lidar a.k.a. RD210MB

  //known issues:
  //1) the longer the lidar is on, the more erratic the output tends to be, might be heat related.  Eg; leaving it on overnight
  //2) lidar tends to crash every so often, the only solution seems to be unplug it and plug it back in
  //3) need to for port to connect before asking for distances

  //Not tested: spamming the Lidar for distances at a high hertz.  might be capped at 7 hertz???

	public:
		// char *portname = "/dev/ttyUSB0";
		RobotLeoLidarDriver(char* portName);

		// returns from an array of distances for each half degree, from [-56, 56) degrees, at half degree resolution.  Distance is in millimeters.
		std::vector<int> getDistances();

		// closes port
		~RobotLeoLidarDriver();

		// in case it's called before port is ready
		bool isReady();

	private:
		char* _portName;
		const int _DataPoints = 112 * 2; //112 deg of FOV, at 0.5deg resolution.
		const int _ReceiveBytesLength = 450; //we expect an ack response of 450 bytes
		int _fd; //fileDescriptor

		// configures port: modified from http://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
		int set_interface_attribs(int fd, int speed, int parity);

		// configures port: modified from http://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
		void set_blocking(int fd, int should_block);
		bool _isReady;
};
