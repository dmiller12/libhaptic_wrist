cmake_minimum_required(VERSION 3.15)

include(FetchContent)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build libraries as static" FORCE)

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

add_library(haptic_wrist STATIC haptic_wrist.cpp trapezoidal_velocity_profile.cpp gravity_comp.cpp kinematics.cpp)
target_include_directories(haptic_wrist PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(haptic_wrist PRIVATE
  $<BUILD_INTERFACE:moteus::cpp>
  $<BUILD_INTERFACE:yaml-cpp::yaml-cpp>
  pthread
  Boost::thread
)

install(TARGETS haptic_wrist
  EXPORT haptic_wristTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY ${CMAKE_SOURCE_DIR}../include/haptic_wrist/ 
  DESTINATION include
)

install(EXPORT haptic_wristTargets
  FILE haptic_wristConfig.cmake
  DESTINATION lib/cmake/haptic_wrist
)

# export(EXPORT haptic_wristTargets FILE ${CMAKE_CURRENT_BINARY_DIR}/haptic_wristTargets.cmake)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/haptic_wristConfigVersion.cmake
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY AnyNewerVersion
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/haptic_wristConfigVersion.cmake
  DESTINATION lib/cmake/haptic_wrist
)

