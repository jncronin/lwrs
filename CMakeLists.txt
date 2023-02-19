cmake_minimum_required(VERSION 3.11 FATAL_ERROR)

project(Lwrs CXX)

add_library(liblwrs)
target_include_directories(liblwrs
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_sources(liblwrs
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src/crc32.cpp")

add_executable(LwrsTest)
target_include_directories(LwrsTest
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_sources(LwrsTest
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src/lwrs_tester.cpp")
target_link_libraries(LwrsTest
    PRIVATE
        liblwrs)
