# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/hao/LidarDriver

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hao/LidarDriver

# Include any dependencies generated for this target.
include CMakeFiles/robotleo_lidar_driver.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/robotleo_lidar_driver.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/robotleo_lidar_driver.dir/flags.make

CMakeFiles/robotleo_lidar_driver.dir/robotleo_lidar_driver.cpp.o: CMakeFiles/robotleo_lidar_driver.dir/flags.make
CMakeFiles/robotleo_lidar_driver.dir/robotleo_lidar_driver.cpp.o: robotleo_lidar_driver.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/hao/LidarDriver/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/robotleo_lidar_driver.dir/robotleo_lidar_driver.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/robotleo_lidar_driver.dir/robotleo_lidar_driver.cpp.o -c /home/hao/LidarDriver/robotleo_lidar_driver.cpp

CMakeFiles/robotleo_lidar_driver.dir/robotleo_lidar_driver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robotleo_lidar_driver.dir/robotleo_lidar_driver.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/hao/LidarDriver/robotleo_lidar_driver.cpp > CMakeFiles/robotleo_lidar_driver.dir/robotleo_lidar_driver.cpp.i

CMakeFiles/robotleo_lidar_driver.dir/robotleo_lidar_driver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robotleo_lidar_driver.dir/robotleo_lidar_driver.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/hao/LidarDriver/robotleo_lidar_driver.cpp -o CMakeFiles/robotleo_lidar_driver.dir/robotleo_lidar_driver.cpp.s

CMakeFiles/robotleo_lidar_driver.dir/robotleo_lidar_driver.cpp.o.requires:
.PHONY : CMakeFiles/robotleo_lidar_driver.dir/robotleo_lidar_driver.cpp.o.requires

CMakeFiles/robotleo_lidar_driver.dir/robotleo_lidar_driver.cpp.o.provides: CMakeFiles/robotleo_lidar_driver.dir/robotleo_lidar_driver.cpp.o.requires
	$(MAKE) -f CMakeFiles/robotleo_lidar_driver.dir/build.make CMakeFiles/robotleo_lidar_driver.dir/robotleo_lidar_driver.cpp.o.provides.build
.PHONY : CMakeFiles/robotleo_lidar_driver.dir/robotleo_lidar_driver.cpp.o.provides

CMakeFiles/robotleo_lidar_driver.dir/robotleo_lidar_driver.cpp.o.provides.build: CMakeFiles/robotleo_lidar_driver.dir/robotleo_lidar_driver.cpp.o

# Object files for target robotleo_lidar_driver
robotleo_lidar_driver_OBJECTS = \
"CMakeFiles/robotleo_lidar_driver.dir/robotleo_lidar_driver.cpp.o"

# External object files for target robotleo_lidar_driver
robotleo_lidar_driver_EXTERNAL_OBJECTS =

librobotleo_lidar_driver.a: CMakeFiles/robotleo_lidar_driver.dir/robotleo_lidar_driver.cpp.o
librobotleo_lidar_driver.a: CMakeFiles/robotleo_lidar_driver.dir/build.make
librobotleo_lidar_driver.a: CMakeFiles/robotleo_lidar_driver.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library librobotleo_lidar_driver.a"
	$(CMAKE_COMMAND) -P CMakeFiles/robotleo_lidar_driver.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/robotleo_lidar_driver.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/robotleo_lidar_driver.dir/build: librobotleo_lidar_driver.a
.PHONY : CMakeFiles/robotleo_lidar_driver.dir/build

CMakeFiles/robotleo_lidar_driver.dir/requires: CMakeFiles/robotleo_lidar_driver.dir/robotleo_lidar_driver.cpp.o.requires
.PHONY : CMakeFiles/robotleo_lidar_driver.dir/requires

CMakeFiles/robotleo_lidar_driver.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/robotleo_lidar_driver.dir/cmake_clean.cmake
.PHONY : CMakeFiles/robotleo_lidar_driver.dir/clean

CMakeFiles/robotleo_lidar_driver.dir/depend:
	cd /home/hao/LidarDriver && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hao/LidarDriver /home/hao/LidarDriver /home/hao/LidarDriver /home/hao/LidarDriver /home/hao/LidarDriver/CMakeFiles/robotleo_lidar_driver.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/robotleo_lidar_driver.dir/depend

