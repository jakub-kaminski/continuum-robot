add_executable(3linkTest
        3linkTest.cpp
        ${CMAKE_SOURCE_DIR}/src/VariableCurvatureRobot.cpp
        )


target_include_directories(3linkTest PRIVATE
        ${CMAKE_SOURCE_DIR}/lib/eigen
        ${CMAKE_SOURCE_DIR}/lib/continuum/include
        ${CMAKE_SOURCE_DIR}/lib/trajectory/include
        ${CMAKE_SOURCE_DIR}/include
        )

target_include_directories(3linkTest PRIVATE
        ${CMAKE_SOURCE_DIR}/lib/eigen
        ${CMAKE_SOURCE_DIR}/lib/continuum/include
        ${CMAKE_SOURCE_DIR}/lib/trajectory/include
        ${CMAKE_SOURCE_DIR}/include
        )

target_link_libraries(3linkTest continuum)
