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
CMAKE_SOURCE_DIR = /home/w/Desktop/Workspace/pcltest/filter

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/w/Desktop/Workspace/pcltest/filter

# Include any dependencies generated for this target.
include CMakeFiles/filter.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/filter.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/filter.dir/flags.make

CMakeFiles/filter.dir/main.cpp.o: CMakeFiles/filter.dir/flags.make
CMakeFiles/filter.dir/main.cpp.o: main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/w/Desktop/Workspace/pcltest/filter/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/filter.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/filter.dir/main.cpp.o -c /home/w/Desktop/Workspace/pcltest/filter/main.cpp

CMakeFiles/filter.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/filter.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/w/Desktop/Workspace/pcltest/filter/main.cpp > CMakeFiles/filter.dir/main.cpp.i

CMakeFiles/filter.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/filter.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/w/Desktop/Workspace/pcltest/filter/main.cpp -o CMakeFiles/filter.dir/main.cpp.s

CMakeFiles/filter.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/filter.dir/main.cpp.o.requires

CMakeFiles/filter.dir/main.cpp.o.provides: CMakeFiles/filter.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/filter.dir/build.make CMakeFiles/filter.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/filter.dir/main.cpp.o.provides

CMakeFiles/filter.dir/main.cpp.o.provides.build: CMakeFiles/filter.dir/main.cpp.o


# Object files for target filter
filter_OBJECTS = \
"CMakeFiles/filter.dir/main.cpp.o"

# External object files for target filter
filter_EXTERNAL_OBJECTS =

filter: CMakeFiles/filter.dir/main.cpp.o
filter: CMakeFiles/filter.dir/build.make
filter: /home/w/freenect2/lib/libfreenect2.so
filter: /usr/local/lib/libopencv_videostab.so.2.4.13
filter: /usr/local/lib/libopencv_ts.a
filter: /usr/local/lib/libopencv_superres.so.2.4.13
filter: /usr/local/lib/libopencv_stitching.so.2.4.13
filter: /usr/local/lib/libopencv_contrib.so.2.4.13
filter: /usr/lib/x86_64-linux-gnu/libboost_system.so
filter: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
filter: /usr/lib/x86_64-linux-gnu/libboost_thread.so
filter: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
filter: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
filter: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
filter: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
filter: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
filter: /usr/lib/x86_64-linux-gnu/libboost_regex.so
filter: /usr/lib/libpcl_common.so
filter: /usr/local/lib/libflann_cpp_s.a
filter: /usr/lib/libpcl_kdtree.so
filter: /usr/lib/libpcl_octree.so
filter: /usr/lib/libpcl_search.so
filter: /usr/lib/x86_64-linux-gnu/libqhull.so
filter: /usr/lib/libpcl_surface.so
filter: /usr/lib/libpcl_sample_consensus.so
filter: /usr/lib/libOpenNI.so
filter: /usr/lib/libOpenNI2.so
filter: /usr/lib/libpcl_io.so
filter: /usr/lib/libpcl_filters.so
filter: /usr/lib/libpcl_features.so
filter: /usr/lib/libpcl_keypoints.so
filter: /usr/lib/libpcl_registration.so
filter: /usr/lib/libpcl_segmentation.so
filter: /usr/lib/libpcl_recognition.so
filter: /usr/lib/libpcl_visualization.so
filter: /usr/lib/libpcl_people.so
filter: /usr/lib/libpcl_outofcore.so
filter: /usr/lib/libpcl_tracking.so
filter: /usr/lib/libpcl_apps.so
filter: /usr/lib/x86_64-linux-gnu/libboost_system.so
filter: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
filter: /usr/lib/x86_64-linux-gnu/libboost_thread.so
filter: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
filter: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
filter: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
filter: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
filter: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
filter: /usr/lib/x86_64-linux-gnu/libboost_regex.so
filter: /usr/lib/x86_64-linux-gnu/libqhull.so
filter: /usr/lib/libOpenNI.so
filter: /usr/lib/libOpenNI2.so
filter: /usr/local/lib/libflann_cpp_s.a
filter: /usr/lib/libvtkGenericFiltering.so.5.8.0
filter: /usr/lib/libvtkGeovis.so.5.8.0
filter: /usr/lib/libvtkCharts.so.5.8.0
filter: /usr/lib/libpcl_common.so
filter: /usr/lib/libpcl_kdtree.so
filter: /usr/lib/libpcl_octree.so
filter: /usr/lib/libpcl_search.so
filter: /usr/lib/libpcl_surface.so
filter: /usr/lib/libpcl_sample_consensus.so
filter: /usr/lib/libpcl_io.so
filter: /usr/lib/libpcl_filters.so
filter: /usr/lib/libpcl_features.so
filter: /usr/lib/libpcl_keypoints.so
filter: /usr/lib/libpcl_registration.so
filter: /usr/lib/libpcl_segmentation.so
filter: /usr/lib/libpcl_recognition.so
filter: /usr/lib/libpcl_visualization.so
filter: /usr/lib/libpcl_people.so
filter: /usr/lib/libpcl_outofcore.so
filter: /usr/lib/libpcl_tracking.so
filter: /usr/lib/libpcl_apps.so
filter: /usr/local/lib/libopencv_nonfree.so.2.4.13
filter: /usr/local/lib/libopencv_ocl.so.2.4.13
filter: /usr/local/lib/libopencv_gpu.so.2.4.13
filter: /usr/local/lib/libopencv_photo.so.2.4.13
filter: /usr/local/lib/libopencv_objdetect.so.2.4.13
filter: /usr/local/lib/libopencv_legacy.so.2.4.13
filter: /usr/local/lib/libopencv_video.so.2.4.13
filter: /usr/local/lib/libopencv_ml.so.2.4.13
filter: /usr/local/lib/libopencv_calib3d.so.2.4.13
filter: /usr/local/lib/libopencv_features2d.so.2.4.13
filter: /usr/local/lib/libopencv_highgui.so.2.4.13
filter: /usr/local/lib/libopencv_imgproc.so.2.4.13
filter: /usr/local/lib/libopencv_flann.so.2.4.13
filter: /usr/local/lib/libopencv_core.so.2.4.13
filter: /usr/lib/libvtkViews.so.5.8.0
filter: /usr/lib/libvtkInfovis.so.5.8.0
filter: /usr/lib/libvtkWidgets.so.5.8.0
filter: /usr/lib/libvtkVolumeRendering.so.5.8.0
filter: /usr/lib/libvtkHybrid.so.5.8.0
filter: /usr/lib/libvtkParallel.so.5.8.0
filter: /usr/lib/libvtkRendering.so.5.8.0
filter: /usr/lib/libvtkImaging.so.5.8.0
filter: /usr/lib/libvtkGraphics.so.5.8.0
filter: /usr/lib/libvtkIO.so.5.8.0
filter: /usr/lib/libvtkFiltering.so.5.8.0
filter: /usr/lib/libvtkCommon.so.5.8.0
filter: /usr/lib/libvtksys.so.5.8.0
filter: CMakeFiles/filter.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/w/Desktop/Workspace/pcltest/filter/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable filter"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/filter.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/filter.dir/build: filter

.PHONY : CMakeFiles/filter.dir/build

CMakeFiles/filter.dir/requires: CMakeFiles/filter.dir/main.cpp.o.requires

.PHONY : CMakeFiles/filter.dir/requires

CMakeFiles/filter.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/filter.dir/cmake_clean.cmake
.PHONY : CMakeFiles/filter.dir/clean

CMakeFiles/filter.dir/depend:
	cd /home/w/Desktop/Workspace/pcltest/filter && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/w/Desktop/Workspace/pcltest/filter /home/w/Desktop/Workspace/pcltest/filter /home/w/Desktop/Workspace/pcltest/filter /home/w/Desktop/Workspace/pcltest/filter /home/w/Desktop/Workspace/pcltest/filter/CMakeFiles/filter.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/filter.dir/depend
