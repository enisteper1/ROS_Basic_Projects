# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_SOURCE_DIR = /home/enis/ROS_Basic_Projects/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/enis/ROS_Basic_Projects/build

# Include any dependencies generated for this target.
include ros_projects/CMakeFiles/area_client.dir/depend.make

# Include the progress variables for this target.
include ros_projects/CMakeFiles/area_client.dir/progress.make

# Include the compile flags for this target's objects.
include ros_projects/CMakeFiles/area_client.dir/flags.make

ros_projects/CMakeFiles/area_client.dir/src/server_client/cpp/area_client.cpp.o: ros_projects/CMakeFiles/area_client.dir/flags.make
ros_projects/CMakeFiles/area_client.dir/src/server_client/cpp/area_client.cpp.o: /home/enis/ROS_Basic_Projects/src/ros_projects/src/server_client/cpp/area_client.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/enis/ROS_Basic_Projects/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ros_projects/CMakeFiles/area_client.dir/src/server_client/cpp/area_client.cpp.o"
	cd /home/enis/ROS_Basic_Projects/build/ros_projects && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/area_client.dir/src/server_client/cpp/area_client.cpp.o -c /home/enis/ROS_Basic_Projects/src/ros_projects/src/server_client/cpp/area_client.cpp

ros_projects/CMakeFiles/area_client.dir/src/server_client/cpp/area_client.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/area_client.dir/src/server_client/cpp/area_client.cpp.i"
	cd /home/enis/ROS_Basic_Projects/build/ros_projects && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/enis/ROS_Basic_Projects/src/ros_projects/src/server_client/cpp/area_client.cpp > CMakeFiles/area_client.dir/src/server_client/cpp/area_client.cpp.i

ros_projects/CMakeFiles/area_client.dir/src/server_client/cpp/area_client.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/area_client.dir/src/server_client/cpp/area_client.cpp.s"
	cd /home/enis/ROS_Basic_Projects/build/ros_projects && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/enis/ROS_Basic_Projects/src/ros_projects/src/server_client/cpp/area_client.cpp -o CMakeFiles/area_client.dir/src/server_client/cpp/area_client.cpp.s

# Object files for target area_client
area_client_OBJECTS = \
"CMakeFiles/area_client.dir/src/server_client/cpp/area_client.cpp.o"

# External object files for target area_client
area_client_EXTERNAL_OBJECTS =

/home/enis/ROS_Basic_Projects/devel/lib/ros_projects/area_client: ros_projects/CMakeFiles/area_client.dir/src/server_client/cpp/area_client.cpp.o
/home/enis/ROS_Basic_Projects/devel/lib/ros_projects/area_client: ros_projects/CMakeFiles/area_client.dir/build.make
/home/enis/ROS_Basic_Projects/devel/lib/ros_projects/area_client: /opt/ros/noetic/lib/libroscpp.so
/home/enis/ROS_Basic_Projects/devel/lib/ros_projects/area_client: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/enis/ROS_Basic_Projects/devel/lib/ros_projects/area_client: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/enis/ROS_Basic_Projects/devel/lib/ros_projects/area_client: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/enis/ROS_Basic_Projects/devel/lib/ros_projects/area_client: /opt/ros/noetic/lib/librosconsole.so
/home/enis/ROS_Basic_Projects/devel/lib/ros_projects/area_client: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/enis/ROS_Basic_Projects/devel/lib/ros_projects/area_client: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/enis/ROS_Basic_Projects/devel/lib/ros_projects/area_client: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/enis/ROS_Basic_Projects/devel/lib/ros_projects/area_client: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/enis/ROS_Basic_Projects/devel/lib/ros_projects/area_client: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/enis/ROS_Basic_Projects/devel/lib/ros_projects/area_client: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/enis/ROS_Basic_Projects/devel/lib/ros_projects/area_client: /opt/ros/noetic/lib/librostime.so
/home/enis/ROS_Basic_Projects/devel/lib/ros_projects/area_client: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/enis/ROS_Basic_Projects/devel/lib/ros_projects/area_client: /opt/ros/noetic/lib/libcpp_common.so
/home/enis/ROS_Basic_Projects/devel/lib/ros_projects/area_client: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/enis/ROS_Basic_Projects/devel/lib/ros_projects/area_client: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/enis/ROS_Basic_Projects/devel/lib/ros_projects/area_client: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/enis/ROS_Basic_Projects/devel/lib/ros_projects/area_client: ros_projects/CMakeFiles/area_client.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/enis/ROS_Basic_Projects/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/enis/ROS_Basic_Projects/devel/lib/ros_projects/area_client"
	cd /home/enis/ROS_Basic_Projects/build/ros_projects && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/area_client.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ros_projects/CMakeFiles/area_client.dir/build: /home/enis/ROS_Basic_Projects/devel/lib/ros_projects/area_client

.PHONY : ros_projects/CMakeFiles/area_client.dir/build

ros_projects/CMakeFiles/area_client.dir/clean:
	cd /home/enis/ROS_Basic_Projects/build/ros_projects && $(CMAKE_COMMAND) -P CMakeFiles/area_client.dir/cmake_clean.cmake
.PHONY : ros_projects/CMakeFiles/area_client.dir/clean

ros_projects/CMakeFiles/area_client.dir/depend:
	cd /home/enis/ROS_Basic_Projects/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/enis/ROS_Basic_Projects/src /home/enis/ROS_Basic_Projects/src/ros_projects /home/enis/ROS_Basic_Projects/build /home/enis/ROS_Basic_Projects/build/ros_projects /home/enis/ROS_Basic_Projects/build/ros_projects/CMakeFiles/area_client.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ros_projects/CMakeFiles/area_client.dir/depend

