CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12.1)
PROJECT(filter_test)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

SET(freenect2_LIBRARIES freenect2)
SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/freenect2/lib/cmake/freenect2)
SET(HDF5_ROOT "/usr/local/hdf5")

FIND_PACKAGE( OpenCV REQUIRED )
FIND_PACKAGE(freenect2 REQUIRED)
FIND_PACKAGE(PCL 1.7 REQUIRED)
FIND_PACKAGE(Flann REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS CXX)

INCLUDE_DIRECTORIES("/usr/include/libusb-1.0/")
INCLUDE_DIRECTORIES(
  ${freenect2_INCLUDE_DIR}  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

INCLUDE_DIRECTORIES(${FLANN_INCLUDE_DIRS})
include_directories(${HDF5_INCLUDE_DIR})

add_library(libmypcl
viewer.cpp filter.cpp segement.cpp normal.cpp VFH.cpp
)

#ADD_EXECUTABLE(pcltest
#  main.cpp 
#)
#TARGET_LINK_LIBRARIES(pcltest
#  ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} libmypcl
#) 

ADD_EXECUTABLE(segtest
  segtest.cpp feature.cpp
)
TARGET_LINK_LIBRARIES(segtest
  ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} libmypcl ${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY}
) 

#ADD_EXECUTABLE(vfhtest
#  vfhtest.cpp 
#)
#TARGET_LINK_LIBRARIES(vfhtest 
#  ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} libmypcl
#) 

#ADD_EXECUTABLE(capture
#  capture.cpp feature.cpp
#)
#TARGET_LINK_LIBRARIES(capture
#  ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} libmypcl ${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY}
#) 

#ADD_EXECUTABLE(viewtest
#  viewtest.cpp
#)
#TARGET_LINK_LIBRARIES(viewtest
#  ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} libmypcl
#) 

ADD_EXECUTABLE(interface
  interface.cpp
)
TARGET_LINK_LIBRARIES(interface
  ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} libmypcl
) 

ADD_EXECUTABLE(buildmodel
  buildmodel.cpp
)
TARGET_LINK_LIBRARIES(buildmodel
  ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} libmypcl
) 
