cmake_minimum_required(VERSION 3.0)

include(FetchContent)
FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG        ee4f6ac9c6391523876836fcf6f276411b7f3670
)
FetchContent_MakeAvailable(moteus)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG "0.8.0"
)
FetchContent_MakeAvailable(yaml-cpp)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

add_library(haptic_wrist haptic_wrist.cpp trapezoidal_velocity_profile.cpp gravity_comp.cpp kinematics.cpp)
target_include_directories(haptic_wrist PUBLIC ../include ${EIGEN3_INCLUDE_DIR})
target_link_libraries(haptic_wrist PUBLIC ${orocos_kdl_LIBRARIES} moteus::cpp pthread Boost::thread yaml-cpp::yaml-cpp)

add_executable(main main.cpp)
target_link_libraries(main haptic_wrist)

