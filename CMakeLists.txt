cmake_minimum_required(VERSION 2.8)

project(NLLS)
add_compile_options("-std=c++11")
SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)
SET(CMAKE_BUILD_TYPE "Debug")


SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})



# test a book
include_directories("/usr/include/eigen3")

include_directories("./src")

add_executable(test_NLLS "./src/main.cpp")

target_link_libraries(test_NLLS ${Pangolin_LIBRARIES})

add_executable(test "./src/test.cpp")

target_link_libraries(test ${Pangolin_LIBRARIES})
