cmake_minimum_required(VERSION 3.11 FATAL_ERROR)

project(LwrsTest CXX)

add_executable(LwrsTest)
target_include_directories(LwrsTest
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_sources(LwrsTest
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src/lwrs_tester.cpp")
