cmake_minimum_required(VERSION 3.5)
project(TEST_CONTROLLER)

include_directories(../../3rd_party)

set(CMAKE_CXX_STANDARD 14)

############################################################################
#############################  TEST EXECUTION ##############################
add_executable(${PROJECT_NAME} test_controller.cpp ../PD/PD.cpp)
#############################  TEST EXECUTION ##############################
############################################################################
