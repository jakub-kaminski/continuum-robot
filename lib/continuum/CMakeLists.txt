cmake_minimum_required(VERSION 3.16)

project(continuumproject)

set(CMAKE_CXX_STANDARD 17)

#include_directories(include/Continuum)
#include_directories(lib/eigen)

add_subdirectory(src)

add_subdirectory(test)
