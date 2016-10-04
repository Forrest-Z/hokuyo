#ifndef _BOB_TOOLBOX_LOG_SETTINGS_H_
#define _BOB_TOOLBOX_LOG_SETTINGS_H_

//! \file logging_settings.h
//! \brief Enable and disable system logging
//! \details This file contains several precompiler defines. Each define enables or disables
//! a specific preprocessor logging macro. Change the value to 0 to disable, and 1 to enable.

//! Enables/Disables LOG_TEMP 
#define LOG_TEMP_ENABLED 1 

//! Enables/Disables LOG_TEST
#define LOG_TEST_ENABLED 1 

//! Enables/Disables LOG_HIGH_LEVEL
#define LOG_HIGH_LEVEL_ENABLED 1

//! Enables/Disables LOG_CONTROL
#define LOG_CONTROL_ENABLED 1

//! Enables/Disables LOG_BOUSTROPHEDON
#define LOG_BOUSTROPHEDON_ENABLED 1

//! Enables/Disables LOG_NAVIGATION
#define LOG_NAVIGATION_ENABLED 1

//! Enables/Disables LOG_SLAM
#define LOG_SLAM_ENABLED 1

#endif
