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
CMAKE_SOURCE_DIR = /home/pat/drone/senior_project/ros/quadrotor/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pat/drone/senior_project/ros/quadrotor/build

# Include any dependencies generated for this target.
include geometry/kdl_conversions/CMakeFiles/kdl_conversions.dir/depend.make

# Include the progress variables for this target.
include geometry/kdl_conversions/CMakeFiles/kdl_conversions.dir/progress.make

# Include the compile flags for this target's objects.
include geometry/kdl_conversions/CMakeFiles/kdl_conversions.dir/flags.make

geometry/kdl_conversions/CMakeFiles/kdl_conversions.dir/src/kdl_msg.cpp.o: geometry/kdl_conversions/CMakeFiles/kdl_conversions.dir/flags.make
geometry/kdl_conversions/CMakeFiles/kdl_conversions.dir/src/kdl_msg.cpp.o: /home/pat/drone/senior_project/ros/quadrotor/src/geometry/kdl_conversions/src/kdl_msg.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pat/drone/senior_project/ros/quadrotor/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object geometry/kdl_conversions/CMakeFiles/kdl_conversions.dir/src/kdl_msg.cpp.o"
	cd /home/pat/drone/senior_project/ros/quadrotor/build/geometry/kdl_conversions && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kdl_conversions.dir/src/kdl_msg.cpp.o -c /home/pat/drone/senior_project/ros/quadrotor/src/geometry/kdl_conversions/src/kdl_msg.cpp

geometry/kdl_conversions/CMakeFiles/kdl_conversions.dir/src/kdl_msg.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kdl_conversions.dir/src/kdl_msg.cpp.i"
	cd /home/pat/drone/senior_project/ros/quadrotor/build/geometry/kdl_conversions && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pat/drone/senior_project/ros/quadrotor/src/geometry/kdl_conversions/src/kdl_msg.cpp > CMakeFiles/kdl_conversions.dir/src/kdl_msg.cpp.i

geometry/kdl_conversions/CMakeFiles/kdl_conversions.dir/src/kdl_msg.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kdl_conversions.dir/src/kdl_msg.cpp.s"
	cd /home/pat/drone/senior_project/ros/quadrotor/build/geometry/kdl_conversions && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pat/drone/senior_project/ros/quadrotor/src/geometry/kdl_conversions/src/kdl_msg.cpp -o CMakeFiles/kdl_conversions.dir/src/kdl_msg.cpp.s

# Object files for target kdl_conversions
kdl_conversions_OBJECTS = \
"CMakeFiles/kdl_conversions.dir/src/kdl_msg.cpp.o"

# External object files for target kdl_conversions
kdl_conversions_EXTERNAL_OBJECTS =

/home/pat/drone/senior_project/ros/quadrotor/devel/lib/libkdl_conversions.so: geometry/kdl_conversions/CMakeFiles/kdl_conversions.dir/src/kdl_msg.cpp.o
/home/pat/drone/senior_project/ros/quadrotor/devel/lib/libkdl_conversions.so: geometry/kdl_conversions/CMakeFiles/kdl_conversions.dir/build.make
/home/pat/drone/senior_project/ros/quadrotor/devel/lib/libkdl_conversions.so: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/pat/drone/senior_project/ros/quadrotor/devel/lib/libkdl_conversions.so: /opt/ros/noetic/lib/librostime.so
/home/pat/drone/senior_project/ros/quadrotor/devel/lib/libkdl_conversions.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/pat/drone/senior_project/ros/quadrotor/devel/lib/libkdl_conversions.so: /opt/ros/noetic/lib/libcpp_common.so
/home/pat/drone/senior_project/ros/quadrotor/devel/lib/libkdl_conversions.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/pat/drone/senior_project/ros/quadrotor/devel/lib/libkdl_conversions.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/pat/drone/senior_project/ros/quadrotor/devel/lib/libkdl_conversions.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/pat/drone/senior_project/ros/quadrotor/devel/lib/libkdl_conversions.so: geometry/kdl_conversions/CMakeFiles/kdl_conversions.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pat/drone/senior_project/ros/quadrotor/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/pat/drone/senior_project/ros/quadrotor/devel/lib/libkdl_conversions.so"
	cd /home/pat/drone/senior_project/ros/quadrotor/build/geometry/kdl_conversions && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kdl_conversions.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
geometry/kdl_conversions/CMakeFiles/kdl_conversions.dir/build: /home/pat/drone/senior_project/ros/quadrotor/devel/lib/libkdl_conversions.so

.PHONY : geometry/kdl_conversions/CMakeFiles/kdl_conversions.dir/build

geometry/kdl_conversions/CMakeFiles/kdl_conversions.dir/clean:
	cd /home/pat/drone/senior_project/ros/quadrotor/build/geometry/kdl_conversions && $(CMAKE_COMMAND) -P CMakeFiles/kdl_conversions.dir/cmake_clean.cmake
.PHONY : geometry/kdl_conversions/CMakeFiles/kdl_conversions.dir/clean

geometry/kdl_conversions/CMakeFiles/kdl_conversions.dir/depend:
	cd /home/pat/drone/senior_project/ros/quadrotor/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pat/drone/senior_project/ros/quadrotor/src /home/pat/drone/senior_project/ros/quadrotor/src/geometry/kdl_conversions /home/pat/drone/senior_project/ros/quadrotor/build /home/pat/drone/senior_project/ros/quadrotor/build/geometry/kdl_conversions /home/pat/drone/senior_project/ros/quadrotor/build/geometry/kdl_conversions/CMakeFiles/kdl_conversions.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : geometry/kdl_conversions/CMakeFiles/kdl_conversions.dir/depend

