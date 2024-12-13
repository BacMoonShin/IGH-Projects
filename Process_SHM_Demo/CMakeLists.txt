cmake_minimum_required(VERSION 3.10)

project(Process_Communication)

set(SRC "temp.cpp")
message(STATUS "Current CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "SRC: ${SRC}")

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 11)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(temp ${SRC})

target_link_libraries(temp pthread)