cmake_minimum_required(VERSION 3.1)
project(dvs_crop_time)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin_simple REQUIRED)
find_package(Glog REQUIRED)
find_package(gflags REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

# make the executable
cs_add_executable(dvs_crop_time
  src/crop_time.cpp
)

# link the executable to the necesarry libs
target_link_libraries(dvs_crop_time
   ${catkin_LIBRARIES}
   ${GLOG_LIBRARIES} ${gflags_LIBRARIES}
)

cs_install()

