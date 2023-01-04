
# COMMENT: target_include_directories is scoped to the given compilation target instead of all subdirectories.
# Using PRIVATE here also makes it not inherited by other, higher CMake projects which is just a good practice.
add_executable(test1 test1.cpp)
target_include_directories(test1 PRIVATE lib/continuum/include )
target_link_libraries(test1 continuum)
