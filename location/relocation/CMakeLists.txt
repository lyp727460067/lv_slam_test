cmake_minimum_required(VERSION 3.5)
project(leetcode)
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../Cmake)
find_package(PCL REQUIRED QUIET)
find_package(Ceres REQUIRED)

set(OpenCV_DIR /home/lyp/third_part/opencv/opencv/build)
#find_package(OpenCV)
find_package(OpenCV  REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)
find_package(GTest  REQUIRED)
find_package(Gflags  REQUIRED)

include_directories(
${PCL_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIR}
${Glog_INLUCDE_DIRS}
${CERES_INCLUDE_DIRS}
${GTSAM_INCLUDE_DIR}
${GTEST_INCLUDE_DIR}
)

link_directories(
	${PCL_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

link_directories(/home/lyp/program/googletest/build/lib/)
#include_directories(${GFLAGS_INCLUDE_DIRS})
set(googtest_lib
${GTEST_LIBRARIES}
${CERES_LIBRARIES}
${GLOG_LIBRARIES}
${GFLAGS_LIBRARIES}
${GLOG_LIBRARIES}
)


file(GLOB_RECURSE ALL_TEST "*_test.cpp")
file(GLOB_RECURSE ALL_LIBRARY_SRCS  "*.cpp")
list(REMOVE_ITEM  ALL_LIBRARY_SRCS ${ALL_TEST} )
file(GLOB_RECURSE ALL_DOTFILES "build/*/*")
list(REMOVE_ITEM  ALL_LIBRARY_SRCS ${ALL_DOTFILES})
file(GLOB_RECURSE COMMON_SRC "common/*.cpp")


 #message(${ALL_LIBRARY_SRCS})
add_library(common_lib ${COMMON_SRC})
foreach (ABS_FIL ${ALL_TEST})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME)
  #message(${FIL_WE})
  # #string(REPLACE "/" "." TEST_TARGET_NAME "${DIR}/${FIL_WE}")
  string(REPLACE ".cpp" ".e" TEST_TARGET_NAME ${REL_FIL})
  # message(${TEST_TARGET_NAME})
  
  add_executable(${TEST_TARGET_NAME}   ${REL_FIL})  
  target_link_libraries(${TEST_TARGET_NAME}   -pthread 
         ${common_lib} ${googtest_lib} )

endforeach()

