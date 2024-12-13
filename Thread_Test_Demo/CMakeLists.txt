cmake_minimum_required(VERSION 3.10)

project(Demo)

set(SRC Thread_Test_Demo.cpp)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_STANDARD 11)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(Thread_Test_Demo ${SRC})

target_link_libraries(Thread_Test_Demo pthread)
