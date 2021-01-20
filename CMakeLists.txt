cmake_minimum_required(VERSION 2.8)

project(NLLS)
add_compile_options("-std=c++11")
SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)
SET(CMAKE_BUILD_TYPE "Debug")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# gtest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# glfags
find_package(gflags REQUIRED)
include_directories(${gflags_INCLUDE_DIR})

# test a book
include_directories("/usr/include/eigen3")

include_directories("./src")

add_executable(test_NLLS "./src/main.cpp")

target_link_libraries(test_NLLS ${Pangolin_LIBRARIES})

# add_executable(test "./src/test.cpp")
#
# target_link_libraries(test ${Pangolin_LIBRARIES})

add_executable(test_glog "./src/test_glog.cpp")
target_link_libraries(test_glog ${GLOG_LIBRARIES} gflags)
