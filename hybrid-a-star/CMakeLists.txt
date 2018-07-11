cmake_minimum_required(VERSION 3.9)
project(hybrid_a_star)

set(CMAKE_CXX_STANDARD 14)

add_executable(hybrid_a_star main.cpp hybrid_breadth_first.cpp hybrid_breadth_first.h)

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

include_directories(/usr/local/include)
link_directories(/usr/local/lib)
