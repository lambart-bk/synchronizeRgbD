cmake_minimum_required(VERSION 2.8.3)
project(synchronizeRgbD)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
  
)
find_package(OpenCV 3 REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(Boost COMPONENTS date_time REQUIRED)
find_package(ZLIB REQUIRED)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
set(BOOST_ALL_DYN_LINK ON)   # force dynamic linking for all libraries


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES savePic
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${ZLIB_INCLUDE_DIR})
include_directories( ${catkin_INCLUDE_DIRS} )

ADD_EXECUTABLE(synchronizeRgbD src/synchronizeRgbD.cpp  )
target_link_libraries(synchronizeRgbD 
				${catkin_LIBRARIES}
				${Boost_LIBRARIES}
				${ZLIB_LIBRARIES}   ) 

