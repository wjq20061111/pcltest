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
CMAKE_SOURCE_DIR = /home/w/Desktop/Workspace/pcltest/datacapture

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/w/Desktop/Workspace/pcltest/datacapture

# Include any dependencies generated for this target.
include CMakeFiles/capture_debug.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/capture_debug.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/capture_debug.dir/flags.make

CMakeFiles/capture_debug.dir/main.cpp.o: CMakeFiles/capture_debug.dir/flags.make
CMakeFiles/capture_debug.dir/main.cpp.o: main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/w/Desktop/Workspace/pcltest/datacapture/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/capture_debug.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/capture_debug.dir/main.cpp.o -c /home/w/Desktop/Workspace/pcltest/datacapture/main.cpp

CMakeFiles/capture_debug.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/capture_debug.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/w/Desktop/Workspace/pcltest/datacapture/main.cpp > CMakeFiles/capture_debug.dir/main.cpp.i

CMakeFiles/capture_debug.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/capture_debug.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/w/Desktop/Workspace/pcltest/datacapture/main.cpp -o CMakeFiles/capture_debug.dir/main.cpp.s

CMakeFiles/capture_debug.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/capture_debug.dir/main.cpp.o.requires

CMakeFiles/capture_debug.dir/main.cpp.o.provides: CMakeFiles/capture_debug.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/capture_debug.dir/build.make CMakeFiles/capture_debug.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/capture_debug.dir/main.cpp.o.provides

CMakeFiles/capture_debug.dir/main.cpp.o.provides.build: CMakeFiles/capture_debug.dir/main.cpp.o


CMakeFiles/capture_debug.dir/filter.cpp.o: CMakeFiles/capture_debug.dir/flags.make
CMakeFiles/capture_debug.dir/filter.cpp.o: filter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/w/Desktop/Workspace/pcltest/datacapture/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/capture_debug.dir/filter.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/capture_debug.dir/filter.cpp.o -c /home/w/Desktop/Workspace/pcltest/datacapture/filter.cpp

CMakeFiles/capture_debug.dir/filter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/capture_debug.dir/filter.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/w/Desktop/Workspace/pcltest/datacapture/filter.cpp > CMakeFiles/capture_debug.dir/filter.cpp.i

CMakeFiles/capture_debug.dir/filter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/capture_debug.dir/filter.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/w/Desktop/Workspace/pcltest/datacapture/filter.cpp -o CMakeFiles/capture_debug.dir/filter.cpp.s

CMakeFiles/capture_debug.dir/filter.cpp.o.requires:

.PHONY : CMakeFiles/capture_debug.dir/filter.cpp.o.requires

CMakeFiles/capture_debug.dir/filter.cpp.o.provides: CMakeFiles/capture_debug.dir/filter.cpp.o.requires
	$(MAKE) -f CMakeFiles/capture_debug.dir/build.make CMakeFiles/capture_debug.dir/filter.cpp.o.provides.build
.PHONY : CMakeFiles/capture_debug.dir/filter.cpp.o.provides

CMakeFiles/capture_debug.dir/filter.cpp.o.provides.build: CMakeFiles/capture_debug.dir/filter.cpp.o


CMakeFiles/capture_debug.dir/viewer.cpp.o: CMakeFiles/capture_debug.dir/flags.make
CMakeFiles/capture_debug.dir/viewer.cpp.o: viewer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/w/Desktop/Workspace/pcltest/datacapture/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/capture_debug.dir/viewer.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/capture_debug.dir/viewer.cpp.o -c /home/w/Desktop/Workspace/pcltest/datacapture/viewer.cpp

CMakeFiles/capture_debug.dir/viewer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/capture_debug.dir/viewer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/w/Desktop/Workspace/pcltest/datacapture/viewer.cpp > CMakeFiles/capture_debug.dir/viewer.cpp.i

CMakeFiles/capture_debug.dir/viewer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/capture_debug.dir/viewer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/w/Desktop/Workspace/pcltest/datacapture/viewer.cpp -o CMakeFiles/capture_debug.dir/viewer.cpp.s

CMakeFiles/capture_debug.dir/viewer.cpp.o.requires:

.PHONY : CMakeFiles/capture_debug.dir/viewer.cpp.o.requires

CMakeFiles/capture_debug.dir/viewer.cpp.o.provides: CMakeFiles/capture_debug.dir/viewer.cpp.o.requires
	$(MAKE) -f CMakeFiles/capture_debug.dir/build.make CMakeFiles/capture_debug.dir/viewer.cpp.o.provides.build
.PHONY : CMakeFiles/capture_debug.dir/viewer.cpp.o.provides

CMakeFiles/capture_debug.dir/viewer.cpp.o.provides.build: CMakeFiles/capture_debug.dir/viewer.cpp.o


# Object files for target capture_debug
capture_debug_OBJECTS = \
"CMakeFiles/capture_debug.dir/main.cpp.o" \
"CMakeFiles/capture_debug.dir/filter.cpp.o" \
"CMakeFiles/capture_debug.dir/viewer.cpp.o"

# External object files for target capture_debug
capture_debug_EXTERNAL_OBJECTS =

capture_debug: CMakeFiles/capture_debug.dir/main.cpp.o
capture_debug: CMakeFiles/capture_debug.dir/filter.cpp.o
capture_debug: CMakeFiles/capture_debug.dir/viewer.cpp.o
capture_debug: CMakeFiles/capture_debug.dir/build.make
capture_debug: /home/w/freenect2/lib/libfreenect2.so
capture_debug: /usr/local/lib/libopencv_videostab.so.2.4.13
capture_debug: /usr/local/lib/libopencv_ts.a
capture_debug: /usr/local/lib/libopencv_superres.so.2.4.13
capture_debug: /usr/local/lib/libopencv_stitching.so.2.4.13
capture_debug: /usr/local/lib/libopencv_contrib.so.2.4.13
capture_debug: /usr/lib/x86_64-linux-gnu/libboost_system.so
capture_debug: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
capture_debug: /usr/lib/x86_64-linux-gnu/libboost_thread.so
capture_debug: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
capture_debug: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
capture_debug: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
capture_debug: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
capture_debug: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
capture_debug: /usr/lib/x86_64-linux-gnu/libboost_regex.so
capture_debug: /usr/lib/libpcl_common.so
capture_debug: /usr/local/lib/libflann_cpp_s.a
capture_debug: /usr/lib/libpcl_kdtree.so
capture_debug: /usr/lib/libpcl_octree.so
capture_debug: /usr/lib/libpcl_search.so
capture_debug: /usr/lib/x86_64-linux-gnu/libqhull.so
capture_debug: /usr/lib/libpcl_surface.so
capture_debug: /usr/lib/libpcl_sample_consensus.so
capture_debug: /usr/lib/libOpenNI.so
capture_debug: /usr/lib/libOpenNI2.so
capture_debug: /usr/lib/libpcl_io.so
capture_debug: /usr/lib/libpcl_filters.so
capture_debug: /usr/lib/libpcl_features.so
capture_debug: /usr/lib/libpcl_keypoints.so
capture_debug: /usr/lib/libpcl_registration.so
capture_debug: /usr/lib/libpcl_segmentation.so
capture_debug: /usr/lib/libpcl_recognition.so
capture_debug: /usr/lib/libpcl_visualization.so
capture_debug: /usr/lib/libpcl_people.so
capture_debug: /usr/lib/libpcl_outofcore.so
capture_debug: /usr/lib/libpcl_tracking.so
capture_debug: /usr/lib/libpcl_apps.so
capture_debug: /usr/lib/x86_64-linux-gnu/libboost_system.so
capture_debug: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
capture_debug: /usr/lib/x86_64-linux-gnu/libboost_thread.so
capture_debug: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
capture_debug: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
capture_debug: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
capture_debug: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
capture_debug: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
capture_debug: /usr/lib/x86_64-linux-gnu/libboost_regex.so
capture_debug: /usr/lib/x86_64-linux-gnu/libqhull.so
capture_debug: /usr/lib/libOpenNI.so
capture_debug: /usr/lib/libOpenNI2.so
capture_debug: /usr/local/lib/libflann_cpp_s.a
capture_debug: /usr/lib/libvtkGenericFiltering.so.5.8.0
capture_debug: /usr/lib/libvtkGeovis.so.5.8.0
capture_debug: /usr/lib/libvtkCharts.so.5.8.0
capture_debug: /usr/lib/libpcl_common.so
capture_debug: /usr/lib/libpcl_kdtree.so
capture_debug: /usr/lib/libpcl_octree.so
capture_debug: /usr/lib/libpcl_search.so
capture_debug: /usr/lib/libpcl_surface.so
capture_debug: /usr/lib/libpcl_sample_consensus.so
capture_debug: /usr/lib/libpcl_io.so
capture_debug: /usr/lib/libpcl_filters.so
capture_debug: /usr/lib/libpcl_features.so
capture_debug: /usr/lib/libpcl_keypoints.so
capture_debug: /usr/lib/libpcl_registration.so
capture_debug: /usr/lib/libpcl_segmentation.so
capture_debug: /usr/lib/libpcl_recognition.so
capture_debug: /usr/lib/libpcl_visualization.so
capture_debug: /usr/lib/libpcl_people.so
capture_debug: /usr/lib/libpcl_outofcore.so
capture_debug: /usr/lib/libpcl_tracking.so
capture_debug: /usr/lib/libpcl_apps.so
capture_debug: /usr/local/lib/libopencv_nonfree.so.2.4.13
capture_debug: /usr/local/lib/libopencv_ocl.so.2.4.13
capture_debug: /usr/local/lib/libopencv_gpu.so.2.4.13
capture_debug: /usr/local/lib/libopencv_photo.so.2.4.13
capture_debug: /usr/local/lib/libopencv_objdetect.so.2.4.13
capture_debug: /usr/local/lib/libopencv_legacy.so.2.4.13
capture_debug: /usr/local/lib/libopencv_video.so.2.4.13
capture_debug: /usr/local/lib/libopencv_ml.so.2.4.13
capture_debug: /usr/local/lib/libopencv_calib3d.so.2.4.13
capture_debug: /usr/local/lib/libopencv_features2d.so.2.4.13
capture_debug: /usr/local/lib/libopencv_highgui.so.2.4.13
capture_debug: /usr/local/lib/libopencv_imgproc.so.2.4.13
capture_debug: /usr/local/lib/libopencv_flann.so.2.4.13
capture_debug: /usr/local/lib/libopencv_core.so.2.4.13
capture_debug: /usr/lib/libvtkViews.so.5.8.0
capture_debug: /usr/lib/libvtkInfovis.so.5.8.0
capture_debug: /usr/lib/libvtkWidgets.so.5.8.0
capture_debug: /usr/lib/libvtkVolumeRendering.so.5.8.0
capture_debug: /usr/lib/libvtkHybrid.so.5.8.0
capture_debug: /usr/lib/libvtkParallel.so.5.8.0
capture_debug: /usr/lib/libvtkRendering.so.5.8.0
capture_debug: /usr/lib/libvtkImaging.so.5.8.0
capture_debug: /usr/lib/libvtkGraphics.so.5.8.0
capture_debug: /usr/lib/libvtkIO.so.5.8.0
capture_debug: /usr/lib/libvtkFiltering.so.5.8.0
capture_debug: /usr/lib/libvtkCommon.so.5.8.0
capture_debug: /usr/lib/libvtksys.so.5.8.0
capture_debug: CMakeFiles/capture_debug.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/w/Desktop/Workspace/pcltest/datacapture/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable capture_debug"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/capture_debug.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/capture_debug.dir/build: capture_debug

.PHONY : CMakeFiles/capture_debug.dir/build

CMakeFiles/capture_debug.dir/requires: CMakeFiles/capture_debug.dir/main.cpp.o.requires
CMakeFiles/capture_debug.dir/requires: CMakeFiles/capture_debug.dir/filter.cpp.o.requires
CMakeFiles/capture_debug.dir/requires: CMakeFiles/capture_debug.dir/viewer.cpp.o.requires

.PHONY : CMakeFiles/capture_debug.dir/requires

CMakeFiles/capture_debug.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/capture_debug.dir/cmake_clean.cmake
.PHONY : CMakeFiles/capture_debug.dir/clean

CMakeFiles/capture_debug.dir/depend:
	cd /home/w/Desktop/Workspace/pcltest/datacapture && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/w/Desktop/Workspace/pcltest/datacapture /home/w/Desktop/Workspace/pcltest/datacapture /home/w/Desktop/Workspace/pcltest/datacapture /home/w/Desktop/Workspace/pcltest/datacapture /home/w/Desktop/Workspace/pcltest/datacapture/CMakeFiles/capture_debug.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/capture_debug.dir/depend

