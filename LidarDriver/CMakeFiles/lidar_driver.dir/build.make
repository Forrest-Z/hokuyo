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
include CMakeFiles/lidar_driver.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/lidar_driver.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/lidar_driver.dir/flags.make

CMakeFiles/lidar_driver.dir/lidar_driver.cpp.o: CMakeFiles/lidar_driver.dir/flags.make
CMakeFiles/lidar_driver.dir/lidar_driver.cpp.o: lidar_driver.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/hao/LidarDriver/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/lidar_driver.dir/lidar_driver.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/lidar_driver.dir/lidar_driver.cpp.o -c /home/hao/LidarDriver/lidar_driver.cpp

CMakeFiles/lidar_driver.dir/lidar_driver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lidar_driver.dir/lidar_driver.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/hao/LidarDriver/lidar_driver.cpp > CMakeFiles/lidar_driver.dir/lidar_driver.cpp.i

CMakeFiles/lidar_driver.dir/lidar_driver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lidar_driver.dir/lidar_driver.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/hao/LidarDriver/lidar_driver.cpp -o CMakeFiles/lidar_driver.dir/lidar_driver.cpp.s

CMakeFiles/lidar_driver.dir/lidar_driver.cpp.o.requires:
.PHONY : CMakeFiles/lidar_driver.dir/lidar_driver.cpp.o.requires

CMakeFiles/lidar_driver.dir/lidar_driver.cpp.o.provides: CMakeFiles/lidar_driver.dir/lidar_driver.cpp.o.requires
	$(MAKE) -f CMakeFiles/lidar_driver.dir/build.make CMakeFiles/lidar_driver.dir/lidar_driver.cpp.o.provides.build
.PHONY : CMakeFiles/lidar_driver.dir/lidar_driver.cpp.o.provides

CMakeFiles/lidar_driver.dir/lidar_driver.cpp.o.provides.build: CMakeFiles/lidar_driver.dir/lidar_driver.cpp.o

# Object files for target lidar_driver
lidar_driver_OBJECTS = \
"CMakeFiles/lidar_driver.dir/lidar_driver.cpp.o"

# External object files for target lidar_driver
lidar_driver_EXTERNAL_OBJECTS =

lidar_driver: CMakeFiles/lidar_driver.dir/lidar_driver.cpp.o
lidar_driver: CMakeFiles/lidar_driver.dir/build.make
lidar_driver: librobotleo_lidar_driver.a
lidar_driver: CMakeFiles/lidar_driver.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable lidar_driver"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lidar_driver.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/lidar_driver.dir/build: lidar_driver
.PHONY : CMakeFiles/lidar_driver.dir/build

CMakeFiles/lidar_driver.dir/requires: CMakeFiles/lidar_driver.dir/lidar_driver.cpp.o.requires
.PHONY : CMakeFiles/lidar_driver.dir/requires

CMakeFiles/lidar_driver.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/lidar_driver.dir/cmake_clean.cmake
.PHONY : CMakeFiles/lidar_driver.dir/clean

CMakeFiles/lidar_driver.dir/depend:
	cd /home/hao/LidarDriver && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hao/LidarDriver /home/hao/LidarDriver /home/hao/LidarDriver /home/hao/LidarDriver /home/hao/LidarDriver/CMakeFiles/lidar_driver.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/lidar_driver.dir/depend

