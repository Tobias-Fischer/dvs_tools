cmake_minimum_required(VERSION 3.1)
project(dvs_filter_bursts)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin_simple REQUIRED)
find_package(Glog REQUIRED)
find_package(gflags REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

cs_add_library(${PROJECT_NAME} src/utils.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GLOG_LIBRARIES} ${gflags_LIBRARIES})

# make the executable
cs_add_executable(filter_bursts src/filter_bursts.cpp)
# link the executable to the project lib
target_link_libraries(filter_bursts ${PROJECT_NAME} ${GLOG_LIBRARIES} ${gflags_LIBRARIES})

cs_install()

