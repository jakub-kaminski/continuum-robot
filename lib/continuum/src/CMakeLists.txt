cmake_minimum_required(VERSION 3.16)

add_library(continuum STATIC
        Continuum.cpp
        Tube.cpp
        ContinuumUtils.cpp
        )

target_include_directories(continuum PRIVATE
        ../lib/eigen
        ../include/Continuum
        )

#target_include_directories(continuum PRIVATE
#        ${CMAKE_SOURCE_DIR}/lib/eigen
#        ${CMAKE_SOURCE_DIR}/include/Continuum
#        )

#include_directories(${CMAKE_SOURCE_DIR}/include/Continuum)
#include_directories(${CMAKE_SOURCE_DIR}/lib/eigen/include)
