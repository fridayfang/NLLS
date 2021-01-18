cmake_minimum_required(VERSION 2.8)

project(NLLS)
add_compile_options("-std=c++11")
SET(CMAKE_BUILD_TYPE "Debug")


SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")




# test a book
include_directories("/usr/include/eigen3")

include_directories("./src")

add_executable(test_NLLS "./src/main.cpp")

