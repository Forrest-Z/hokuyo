/*
 * RobotLeoLidarDriver.cpp
 *
 *  Created on: Sep 9, 2016
 *      Author: peter
 */
#include <stdio.h> // standard input / output functions// string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitions
#include <time.h>   // time calls
#include <iostream>

#include <cstdio>
#include <cstdlib>
#include <string.h>
#include "robotleo_lidar_driver.hpp"

int RobotLeoLidarDriver::set_interface_attribs(int fd, int speed, int parity)
{
	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0)
	{
		printf("Lidar error %d from tcgetattr.  Are you running in root?\n", errno);
		return -1;
	}

	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK; // disable break processing
	tty.c_lflag = 0; // no signaling chars, no echo,
					 // no canonical processing
	tty.c_oflag = 0; // no remapping, no delays
	tty.c_cc[VMIN] = 1; // read doesn't block
	tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
									 // enable reading
	tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB; //one bit
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr(fd, TCSANOW, &tty) != 0)
	{
		printf("Lidar error %d from tcsetattr.  Are you running in root?\n", errno);
		return -1;
	}
	return 0;
}

RobotLeoLidarDriver::~RobotLeoLidarDriver()
{
	close(_fd);
}

RobotLeoLidarDriver::RobotLeoLidarDriver(char* portName)
{
	_isReady = false;

	_portName = portName;
	_fd = open(portName, O_RDWR | O_NOCTTY | O_SYNC);
	if (_fd < 0)
	{
		printf("Lidar Error: %d opening %s: %s\n", errno, _portName, strerror(errno));

		//TODO: maybe try to re-open port at a later time?
	}

	set_interface_attribs(_fd, B115200, 0); // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking(_fd, 1); // set blocking

	_isReady = true;
}

bool RobotLeoLidarDriver::isReady()
{
	return _isReady;
}

std::vector<int> RobotLeoLidarDriver::getDistances()
{

	if (_isReady == false)
		return std::vector<int>();

	const int send_bytes_length = 7;
	unsigned char send_bytes[send_bytes_length] = { 0xA5, 0x07, 0x01, 0x36, 0x00, 0x00, 0xE3 }; //this is from the manual

	/*	for (int i = 0; i < send_bytes_length; i++)
	 printf("%c ", send_bytes[i]);
	 printf("\n");*/

	tcflush(_fd, TCIOFLUSH); //flush buffer so we'll know what's in the buffer is a result of our write.

	write(_fd, send_bytes, send_bytes_length); //Send data

	//printf("I'm done writing\n");

	unsigned char buf[_ReceiveBytesLength];

	int n = read(_fd, buf, _ReceiveBytesLength); // read characters if ready to read

	if (buf[0] == send_bytes[0] && buf[1] == 0x01)
		; //printf("A OK\n");
	else
	{
		printf("Lidar Error: incorrect ack response from Lidar.\n ");
		//close (_fd);
		//return std::vector<int>();
	}

	int totalReceivedBytes = n;
	int halfDegreeIter = 0;
	int skippedBytes = 0;

	std::vector<int> rangingDistances(_DataPoints);

	while (true)
	{ //|| n > 0) {  || n > 0 is needed cuz sometimes we start with an non-empty buffer

		unsigned char hexadecimalOutput[2];

		for (int i = 0; i < n; i++)
		{
			if (totalReceivedBytes == n && skippedBytes < 2)
			{ //skip the first 2 bytes of the first read because it's just the ack
				skippedBytes++;
				continue;
			}

			if (i % 2 == 1)
			{
				hexadecimalOutput[1] = buf[i];
				int distance = (int) hexadecimalOutput[0] * 256 + (int) hexadecimalOutput[1];
				//	printf("%.1f %d, ", (halfDegreeIter) * 0.5, distance);
				rangingDistances[halfDegreeIter] = distance;
				halfDegreeIter++;
			}
			else
			{
				hexadecimalOutput[0] = buf[i];
				//printf(" ");
			}
		}

		if (totalReceivedBytes >= _ReceiveBytesLength)
			break;

		n = read(_fd, buf, sizeof buf);
		totalReceivedBytes += n;
	}

	/*	for (int i = 0; i < dataPoints; i++)
	 printf("%.1f %d\n", i * 0.5, rangingDistances[i]);*/

	return rangingDistances;
}

void RobotLeoLidarDriver::set_blocking(int fd, int should_block)
{
	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0)
	{
		printf("Lidar error %d from tggetattr\n", errno);
		return;
	}

	tty.c_cc[VMIN] = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

	if (tcsetattr(fd, TCSANOW, &tty) != 0)
		printf("Lidar error %d setting term attributes\n", errno);
}
