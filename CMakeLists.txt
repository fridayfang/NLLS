cmake_minimum_required(VERSION 2.8)

project(NLLS)

add_compile_options("-std=c++14") 
# unique_ptr 需要 c++14


# test a book
include_directories("/usr/include/eigen3")

include_directories("./src")

add_executable(test_NLLS "./src/main.cpp")

