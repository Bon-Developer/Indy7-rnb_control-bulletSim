cmake_minimum_required(VERSION 3.5)
project(PD)

include_directories(../../3rd_party)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_SHARED_LIBRARY_PREFIX "") # output file name should match with BOOST_PYTHON_MODULE name

add_library(${PROJECT_NAME} SHARED ${PROJECT_NAME}.cpp)
