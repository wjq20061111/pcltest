CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12.1)
PROJECT(libfreenect2_opencv_test)
#SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11")
SET(freenect2_LIBRARIES freenect2)
SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/freenect2/lib/cmake/freenect2)


FIND_PACKAGE( OpenCV REQUIRED )
FIND_PACKAGE(freenect2 REQUIRED)
FIND_PACKAGE(PCL 1.7 REQUIRED)

INCLUDE_DIRECTORIES("/usr/include/libusb-1.0/")
INCLUDE_DIRECTORIES(
  ${freenect2_INCLUDE_DIR}  ${PCL_INCLUDE_DIRS}
)
INCLUDE_DIRECTORIES("iai/")


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


ADD_EXECUTABLE(ptest
  main.cpp
)
TARGET_LINK_LIBRARIES(ptest
  ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES}
) 

ADD_EXECUTABLE(interface
  interface.cpp iai/depth_registration_cpu.cpp iai/kinect2_registration.cpp
)
TARGET_LINK_LIBRARIES(interface
  ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES}
) 
