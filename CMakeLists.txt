cmake_minimum_required(VERSION 3.8)

project(variable-curvature-robot)

set(CMAKE_VERBOSE_MAKEFILE on)

set(CMAKE_CXX_STANDARD 17)

#uncomment
#set(CMAKE_EXE_LINKER_FLAGS "-s USE_BOOST_HEADERS=1")

add_subdirectory(lib)
add_subdirectory(src)
add_subdirectory(wasm)
add_subdirectory(test)

