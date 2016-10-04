#ifndef _BOB_TOOLBOX_LOG_H_ 
#define _BOB_TOOLBOX_LOG_H_

//! \file logging.h
//! \brief Defines several preprocessor macros used for logging.
//! \details These macros are designed to be used with the stream operator, like std::cout.
//! This is useful when combined with the stream operator functions in easy_print.h. \n
//! Here is an example: \n
//! \code {.cpp} LOG_TEST("Val:" << 3 << " Loc: " << MapLocation(2, 0));\endcode  \n
//! You can enable/disable each individual macro in logging_settings.h. You will need to
//! recompile before it takes effect. \n
//! Feel free to add your own logging macros when you think there is a need for them. 
//! Just need to copy the style that's already laid out in this file, and in logging_settings.h.


#include <string>
#include <iostream>
#include <bob_toolbox/easy_print.h>
#include <bob_toolbox/logging_settings.h> 

//! For casual use while debugging. It will eventually be removed, or changed to another type.
#if LOG_TEMP_ENABLED			 
	#define LOG_TEMP(text) std::cout << "[TEMP]: " << text << std::endl;
#else						 
	#define LOG_TEMP(text) 		 
#endif

//! Used for short tests, such as in bob_test
#if LOG_TEST_ENABLED			 
	#define LOG_TEST(text) std::cout << "[TEST]: " << text << std::endl;
#else						 
	#define LOG_TEST(text) 		 
#endif

//! Used for high-level functions of the robot (switching between behavior modes, etc.)
#if LOG_HIGH_LEVEL_ENABLED			 
	#define LOG_HIGH_LEVEL(text) std::cout << "[HIGH_LEVEL]: " << text << std::endl;
#else						 
	#define LOG_HIGH_LEVEL(text) 		 
#endif

//! Used for control systems
#if LOG_CONTROL_ENABLED			 
	#define LOG_CONTROL(text) std::cout << "[CONTROL]: " << text << std::endl;
#else						 
	#define LOG_CONTROL(text) 		 
#endif

//! Boustrophedon coverage system
#if LOG_BOUSTROPHEDON_ENABLED			 
	#define LOG_BOUSTROPHEDON(text) std::cout << "[BOUSTROPHEDON]: " << text << std::endl;
#else						 
	#define LOG_BOUSTROPHEDON(text) 
#endif

//! Navigation system
#if LOG_NAVIGATION_ENABLED			 
	#define LOG_NAVIGATION(text) std::cout << "[NAVIGATION]: " << text << std::endl;
#else						 
	#define LOG_NAVIGATION(text) 
#endif

//! SLAM system
#if LOG_SLAM_ENABLED			 
	#define LOG_SLAM(text) std::cout << "[SLAM]: " << text << std::endl;
#else						 
	#define LOG_SLAM(text) 
#endif

#define LOG_ERROR(text) std::cout << "[ERROR]: " << text << std::endl;

#endif
