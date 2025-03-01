cmake_minimum_required(VERSION 3.0)
project(haptic_wrist)

include(FetchContent)
FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG        ee4f6ac9c6391523876836fcf6f276411b7f3670
)
FetchContent_MakeAvailable(moteus)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

add_library(haptic_wrist haptic_wrist.cpp)
target_include_directories(haptic_wrist PUBLIC ../include ${Eigen3_INCLUDE_DIRS})
target_link_libraries(haptic_wrist orocos_kdl moteus::cpp Boost:thread)
