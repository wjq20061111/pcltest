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
CMAKE_SOURCE_DIR = /home/w/Desktop/Workspace/pcltest/test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/w/Desktop/Workspace/pcltest/test

# Include any dependencies generated for this target.
include CMakeFiles/segtest.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/segtest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/segtest.dir/flags.make

CMakeFiles/segtest.dir/segtest.cpp.o: CMakeFiles/segtest.dir/flags.make
CMakeFiles/segtest.dir/segtest.cpp.o: segtest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/w/Desktop/Workspace/pcltest/test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/segtest.dir/segtest.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/segtest.dir/segtest.cpp.o -c /home/w/Desktop/Workspace/pcltest/test/segtest.cpp

CMakeFiles/segtest.dir/segtest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/segtest.dir/segtest.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/w/Desktop/Workspace/pcltest/test/segtest.cpp > CMakeFiles/segtest.dir/segtest.cpp.i

CMakeFiles/segtest.dir/segtest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/segtest.dir/segtest.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/w/Desktop/Workspace/pcltest/test/segtest.cpp -o CMakeFiles/segtest.dir/segtest.cpp.s

CMakeFiles/segtest.dir/segtest.cpp.o.requires:

.PHONY : CMakeFiles/segtest.dir/segtest.cpp.o.requires

CMakeFiles/segtest.dir/segtest.cpp.o.provides: CMakeFiles/segtest.dir/segtest.cpp.o.requires
	$(MAKE) -f CMakeFiles/segtest.dir/build.make CMakeFiles/segtest.dir/segtest.cpp.o.provides.build
.PHONY : CMakeFiles/segtest.dir/segtest.cpp.o.provides

CMakeFiles/segtest.dir/segtest.cpp.o.provides.build: CMakeFiles/segtest.dir/segtest.cpp.o


CMakeFiles/segtest.dir/feature.cpp.o: CMakeFiles/segtest.dir/flags.make
CMakeFiles/segtest.dir/feature.cpp.o: feature.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/w/Desktop/Workspace/pcltest/test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/segtest.dir/feature.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/segtest.dir/feature.cpp.o -c /home/w/Desktop/Workspace/pcltest/test/feature.cpp

CMakeFiles/segtest.dir/feature.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/segtest.dir/feature.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/w/Desktop/Workspace/pcltest/test/feature.cpp > CMakeFiles/segtest.dir/feature.cpp.i

CMakeFiles/segtest.dir/feature.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/segtest.dir/feature.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/w/Desktop/Workspace/pcltest/test/feature.cpp -o CMakeFiles/segtest.dir/feature.cpp.s

CMakeFiles/segtest.dir/feature.cpp.o.requires:

.PHONY : CMakeFiles/segtest.dir/feature.cpp.o.requires

CMakeFiles/segtest.dir/feature.cpp.o.provides: CMakeFiles/segtest.dir/feature.cpp.o.requires
	$(MAKE) -f CMakeFiles/segtest.dir/build.make CMakeFiles/segtest.dir/feature.cpp.o.provides.build
.PHONY : CMakeFiles/segtest.dir/feature.cpp.o.provides

CMakeFiles/segtest.dir/feature.cpp.o.provides.build: CMakeFiles/segtest.dir/feature.cpp.o


# Object files for target segtest
segtest_OBJECTS = \
"CMakeFiles/segtest.dir/segtest.cpp.o" \
"CMakeFiles/segtest.dir/feature.cpp.o"

# External object files for target segtest
segtest_EXTERNAL_OBJECTS =

segtest: CMakeFiles/segtest.dir/segtest.cpp.o
segtest: CMakeFiles/segtest.dir/feature.cpp.o
segtest: CMakeFiles/segtest.dir/build.make
segtest: /home/w/freenect2/lib/libfreenect2.so
segtest: /usr/local/lib/libopencv_videostab.so.2.4.13
segtest: /usr/local/lib/libopencv_ts.a
segtest: /usr/local/lib/libopencv_superres.so.2.4.13
segtest: /usr/local/lib/libopencv_stitching.so.2.4.13
segtest: /usr/local/lib/libopencv_contrib.so.2.4.13
segtest: /usr/lib/x86_64-linux-gnu/libboost_system.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_thread.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_regex.so
segtest: /usr/lib/libpcl_common.so
segtest: /usr/local/lib/libflann_cpp_s.a
segtest: /usr/lib/libpcl_kdtree.so
segtest: /usr/lib/libpcl_octree.so
segtest: /usr/lib/libpcl_search.so
segtest: /usr/lib/x86_64-linux-gnu/libqhull.so
segtest: /usr/lib/libpcl_surface.so
segtest: /usr/lib/libpcl_sample_consensus.so
segtest: /usr/lib/libOpenNI.so
segtest: /usr/lib/libOpenNI2.so
segtest: /usr/lib/libpcl_io.so
segtest: /usr/lib/libpcl_filters.so
segtest: /usr/lib/libpcl_features.so
segtest: /usr/lib/libpcl_keypoints.so
segtest: /usr/lib/libpcl_registration.so
segtest: /usr/lib/libpcl_segmentation.so
segtest: /usr/lib/libpcl_recognition.so
segtest: /usr/lib/libpcl_visualization.so
segtest: /usr/lib/libpcl_people.so
segtest: /usr/lib/libpcl_outofcore.so
segtest: /usr/lib/libpcl_tracking.so
segtest: /usr/lib/libpcl_apps.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_system.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_thread.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_regex.so
segtest: /usr/lib/x86_64-linux-gnu/libqhull.so
segtest: /usr/lib/libOpenNI.so
segtest: /usr/lib/libOpenNI2.so
segtest: /usr/local/lib/libflann_cpp_s.a
segtest: /usr/lib/libvtkGenericFiltering.so.5.8.0
segtest: /usr/lib/libvtkGeovis.so.5.8.0
segtest: /usr/lib/libvtkCharts.so.5.8.0
segtest: liblibmypcl.a
segtest: /usr/lib/x86_64-linux-gnu/libboost_system.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_thread.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
segtest: /usr/lib/x86_64-linux-gnu/libboost_regex.so
segtest: /usr/local/lib/libflann_cpp_s.a
segtest: /usr/local/hdf5/lib/libhdf5.so
segtest: /usr/lib/libpcl_common.so
segtest: /usr/lib/libpcl_kdtree.so
segtest: /usr/lib/libpcl_octree.so
segtest: /usr/lib/libpcl_search.so
segtest: /usr/lib/libpcl_surface.so
segtest: /usr/lib/libpcl_sample_consensus.so
segtest: /usr/lib/libpcl_io.so
segtest: /usr/lib/libpcl_filters.so
segtest: /usr/lib/libpcl_features.so
segtest: /usr/lib/libpcl_keypoints.so
segtest: /usr/lib/libpcl_registration.so
segtest: /usr/lib/libpcl_segmentation.so
segtest: /usr/lib/libpcl_recognition.so
segtest: /usr/lib/libpcl_visualization.so
segtest: /usr/lib/libpcl_people.so
segtest: /usr/lib/libpcl_outofcore.so
segtest: /usr/lib/libpcl_tracking.so
segtest: /usr/lib/libpcl_apps.so
segtest: /usr/local/hdf5/lib/libhdf5.so
segtest: /usr/local/lib/libopencv_nonfree.so.2.4.13
segtest: /usr/local/lib/libopencv_ocl.so.2.4.13
segtest: /usr/local/lib/libopencv_gpu.so.2.4.13
segtest: /usr/local/lib/libopencv_photo.so.2.4.13
segtest: /usr/local/lib/libopencv_objdetect.so.2.4.13
segtest: /usr/local/lib/libopencv_legacy.so.2.4.13
segtest: /usr/local/lib/libopencv_video.so.2.4.13
segtest: /usr/local/lib/libopencv_ml.so.2.4.13
segtest: /usr/local/lib/libopencv_calib3d.so.2.4.13
segtest: /usr/local/lib/libopencv_features2d.so.2.4.13
segtest: /usr/local/lib/libopencv_highgui.so.2.4.13
segtest: /usr/local/lib/libopencv_imgproc.so.2.4.13
segtest: /usr/local/lib/libopencv_flann.so.2.4.13
segtest: /usr/local/lib/libopencv_core.so.2.4.13
segtest: /usr/lib/libvtkViews.so.5.8.0
segtest: /usr/lib/libvtkInfovis.so.5.8.0
segtest: /usr/lib/libvtkWidgets.so.5.8.0
segtest: /usr/lib/libvtkVolumeRendering.so.5.8.0
segtest: /usr/lib/libvtkHybrid.so.5.8.0
segtest: /usr/lib/libvtkParallel.so.5.8.0
segtest: /usr/lib/libvtkRendering.so.5.8.0
segtest: /usr/lib/libvtkImaging.so.5.8.0
segtest: /usr/lib/libvtkGraphics.so.5.8.0
segtest: /usr/lib/libvtkIO.so.5.8.0
segtest: /usr/lib/libvtkFiltering.so.5.8.0
segtest: /usr/lib/libvtkCommon.so.5.8.0
segtest: /usr/lib/libvtksys.so.5.8.0
segtest: CMakeFiles/segtest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/w/Desktop/Workspace/pcltest/test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable segtest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/segtest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/segtest.dir/build: segtest

.PHONY : CMakeFiles/segtest.dir/build

CMakeFiles/segtest.dir/requires: CMakeFiles/segtest.dir/segtest.cpp.o.requires
CMakeFiles/segtest.dir/requires: CMakeFiles/segtest.dir/feature.cpp.o.requires

.PHONY : CMakeFiles/segtest.dir/requires

CMakeFiles/segtest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/segtest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/segtest.dir/clean

CMakeFiles/segtest.dir/depend:
	cd /home/w/Desktop/Workspace/pcltest/test && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/w/Desktop/Workspace/pcltest/test /home/w/Desktop/Workspace/pcltest/test /home/w/Desktop/Workspace/pcltest/test /home/w/Desktop/Workspace/pcltest/test /home/w/Desktop/Workspace/pcltest/test/CMakeFiles/segtest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/segtest.dir/depend

