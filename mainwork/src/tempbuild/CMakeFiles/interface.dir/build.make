# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.8

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/w/Desktop/Workspace/pcltest/mainwork/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/w/Desktop/Workspace/pcltest/mainwork/src/tempbuild

# Include any dependencies generated for this target.
include CMakeFiles/interface.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/interface.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/interface.dir/flags.make

CMakeFiles/interface.dir/interface.cpp.o: CMakeFiles/interface.dir/flags.make
CMakeFiles/interface.dir/interface.cpp.o: ../interface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/w/Desktop/Workspace/pcltest/mainwork/src/tempbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/interface.dir/interface.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/interface.dir/interface.cpp.o -c /home/w/Desktop/Workspace/pcltest/mainwork/src/interface.cpp

CMakeFiles/interface.dir/interface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/interface.dir/interface.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/w/Desktop/Workspace/pcltest/mainwork/src/interface.cpp > CMakeFiles/interface.dir/interface.cpp.i

CMakeFiles/interface.dir/interface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/interface.dir/interface.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/w/Desktop/Workspace/pcltest/mainwork/src/interface.cpp -o CMakeFiles/interface.dir/interface.cpp.s

CMakeFiles/interface.dir/interface.cpp.o.requires:

.PHONY : CMakeFiles/interface.dir/interface.cpp.o.requires

CMakeFiles/interface.dir/interface.cpp.o.provides: CMakeFiles/interface.dir/interface.cpp.o.requires
	$(MAKE) -f CMakeFiles/interface.dir/build.make CMakeFiles/interface.dir/interface.cpp.o.provides.build
.PHONY : CMakeFiles/interface.dir/interface.cpp.o.provides

CMakeFiles/interface.dir/interface.cpp.o.provides.build: CMakeFiles/interface.dir/interface.cpp.o


CMakeFiles/interface.dir/iai/depth_registration_cpu.cpp.o: CMakeFiles/interface.dir/flags.make
CMakeFiles/interface.dir/iai/depth_registration_cpu.cpp.o: ../iai/depth_registration_cpu.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/w/Desktop/Workspace/pcltest/mainwork/src/tempbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/interface.dir/iai/depth_registration_cpu.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/interface.dir/iai/depth_registration_cpu.cpp.o -c /home/w/Desktop/Workspace/pcltest/mainwork/src/iai/depth_registration_cpu.cpp

CMakeFiles/interface.dir/iai/depth_registration_cpu.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/interface.dir/iai/depth_registration_cpu.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/w/Desktop/Workspace/pcltest/mainwork/src/iai/depth_registration_cpu.cpp > CMakeFiles/interface.dir/iai/depth_registration_cpu.cpp.i

CMakeFiles/interface.dir/iai/depth_registration_cpu.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/interface.dir/iai/depth_registration_cpu.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/w/Desktop/Workspace/pcltest/mainwork/src/iai/depth_registration_cpu.cpp -o CMakeFiles/interface.dir/iai/depth_registration_cpu.cpp.s

CMakeFiles/interface.dir/iai/depth_registration_cpu.cpp.o.requires:

.PHONY : CMakeFiles/interface.dir/iai/depth_registration_cpu.cpp.o.requires

CMakeFiles/interface.dir/iai/depth_registration_cpu.cpp.o.provides: CMakeFiles/interface.dir/iai/depth_registration_cpu.cpp.o.requires
	$(MAKE) -f CMakeFiles/interface.dir/build.make CMakeFiles/interface.dir/iai/depth_registration_cpu.cpp.o.provides.build
.PHONY : CMakeFiles/interface.dir/iai/depth_registration_cpu.cpp.o.provides

CMakeFiles/interface.dir/iai/depth_registration_cpu.cpp.o.provides.build: CMakeFiles/interface.dir/iai/depth_registration_cpu.cpp.o


CMakeFiles/interface.dir/iai/kinect2_registration.cpp.o: CMakeFiles/interface.dir/flags.make
CMakeFiles/interface.dir/iai/kinect2_registration.cpp.o: ../iai/kinect2_registration.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/w/Desktop/Workspace/pcltest/mainwork/src/tempbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/interface.dir/iai/kinect2_registration.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/interface.dir/iai/kinect2_registration.cpp.o -c /home/w/Desktop/Workspace/pcltest/mainwork/src/iai/kinect2_registration.cpp

CMakeFiles/interface.dir/iai/kinect2_registration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/interface.dir/iai/kinect2_registration.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/w/Desktop/Workspace/pcltest/mainwork/src/iai/kinect2_registration.cpp > CMakeFiles/interface.dir/iai/kinect2_registration.cpp.i

CMakeFiles/interface.dir/iai/kinect2_registration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/interface.dir/iai/kinect2_registration.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/w/Desktop/Workspace/pcltest/mainwork/src/iai/kinect2_registration.cpp -o CMakeFiles/interface.dir/iai/kinect2_registration.cpp.s

CMakeFiles/interface.dir/iai/kinect2_registration.cpp.o.requires:

.PHONY : CMakeFiles/interface.dir/iai/kinect2_registration.cpp.o.requires

CMakeFiles/interface.dir/iai/kinect2_registration.cpp.o.provides: CMakeFiles/interface.dir/iai/kinect2_registration.cpp.o.requires
	$(MAKE) -f CMakeFiles/interface.dir/build.make CMakeFiles/interface.dir/iai/kinect2_registration.cpp.o.provides.build
.PHONY : CMakeFiles/interface.dir/iai/kinect2_registration.cpp.o.provides

CMakeFiles/interface.dir/iai/kinect2_registration.cpp.o.provides.build: CMakeFiles/interface.dir/iai/kinect2_registration.cpp.o


# Object files for target interface
interface_OBJECTS = \
"CMakeFiles/interface.dir/interface.cpp.o" \
"CMakeFiles/interface.dir/iai/depth_registration_cpu.cpp.o" \
"CMakeFiles/interface.dir/iai/kinect2_registration.cpp.o"

# External object files for target interface
interface_EXTERNAL_OBJECTS =

/home/w/Desktop/Workspace/pcltest/mainwork/lib/libinterface.a: CMakeFiles/interface.dir/interface.cpp.o
/home/w/Desktop/Workspace/pcltest/mainwork/lib/libinterface.a: CMakeFiles/interface.dir/iai/depth_registration_cpu.cpp.o
/home/w/Desktop/Workspace/pcltest/mainwork/lib/libinterface.a: CMakeFiles/interface.dir/iai/kinect2_registration.cpp.o
/home/w/Desktop/Workspace/pcltest/mainwork/lib/libinterface.a: CMakeFiles/interface.dir/build.make
/home/w/Desktop/Workspace/pcltest/mainwork/lib/libinterface.a: CMakeFiles/interface.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/w/Desktop/Workspace/pcltest/mainwork/src/tempbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library /home/w/Desktop/Workspace/pcltest/mainwork/lib/libinterface.a"
	$(CMAKE_COMMAND) -P CMakeFiles/interface.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/interface.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/interface.dir/build: /home/w/Desktop/Workspace/pcltest/mainwork/lib/libinterface.a

.PHONY : CMakeFiles/interface.dir/build

CMakeFiles/interface.dir/requires: CMakeFiles/interface.dir/interface.cpp.o.requires
CMakeFiles/interface.dir/requires: CMakeFiles/interface.dir/iai/depth_registration_cpu.cpp.o.requires
CMakeFiles/interface.dir/requires: CMakeFiles/interface.dir/iai/kinect2_registration.cpp.o.requires

.PHONY : CMakeFiles/interface.dir/requires

CMakeFiles/interface.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/interface.dir/cmake_clean.cmake
.PHONY : CMakeFiles/interface.dir/clean

CMakeFiles/interface.dir/depend:
	cd /home/w/Desktop/Workspace/pcltest/mainwork/src/tempbuild && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/w/Desktop/Workspace/pcltest/mainwork/src /home/w/Desktop/Workspace/pcltest/mainwork/src /home/w/Desktop/Workspace/pcltest/mainwork/src/tempbuild /home/w/Desktop/Workspace/pcltest/mainwork/src/tempbuild /home/w/Desktop/Workspace/pcltest/mainwork/src/tempbuild/CMakeFiles/interface.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/interface.dir/depend
