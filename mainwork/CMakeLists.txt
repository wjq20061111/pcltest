CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12.1)
PROJECT(filter_test)

SET(freenect2_LIBRARIES freenect2)
SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/freenect2/lib/cmake/freenect2)


FIND_PACKAGE( OpenCV REQUIRED )
FIND_PACKAGE(freenect2 REQUIRED)
FIND_PACKAGE(PCL 1.7 REQUIRED)

INCLUDE_DIRECTORIES("/usr/include/libusb-1.0/")
INCLUDE_DIRECTORIES(
  ${freenect2_INCLUDE_DIR}  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ADD_EXECUTABLE(pcltest
  main.cpp viewer.cpp filter.cpp segement.cpp
)
TARGET_LINK_LIBRARIES(pcltest
  ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES}
) 
