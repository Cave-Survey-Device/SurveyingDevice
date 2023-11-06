cmake_minimum_required(VERSION 3.24)
project(NumericalMethodsTesting)

set(CMAKE_CXX_STANDARD 17)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/include")
include_directories(include)

add_executable(NumericalMethodsTesting main.cpp)
