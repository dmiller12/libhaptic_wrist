cmake_minimum_required(VERSION 3.15)
project(haptic_wrist VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

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
find_package(Boost COMPONENTS thread filesystem REQUIRED)

add_library(haptic_wrist SHARED)
target_sources(haptic_wrist
	PRIVATE
	src/haptic_wrist_impl.cpp
	src/haptic_wrist.cpp
	src/trapezoidal_velocity_profile.cpp
	src/gravity_comp.cpp
	src/kinematics.cpp
	src/utils.cpp
)
target_include_directories(haptic_wrist PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(haptic_wrist PUBLIC
  Boost::thread Boost::filesystem pthread
)

target_link_libraries(haptic_wrist PRIVATE
  moteus::cpp yaml-cpp 
)

install(TARGETS haptic_wrist
  EXPORT haptic_wristTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/haptic_wrist/ 
  DESTINATION include/haptic_wrist
)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/config/
    DESTINATION /etc/haptic_wrist
    FILES_MATCHING PATTERN "*.yaml"  # Adjust the pattern if needed
    PERMISSIONS
        OWNER_READ OWNER_WRITE OWNER_EXECUTE
	GROUP_READ GROUP_WRITE GROUP_EXECUTE
	WORLD_READ WORLD_WRITE WORLD_EXECUTE
)

install(EXPORT haptic_wristTargets
  DESTINATION lib/cmake/haptic_wrist
)


export(TARGETS haptic_wrist FILE haptic_wristTargets.cmake)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/haptic_wristConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/haptic_wristConfig.cmake
  INSTALL_DESTINATION lib/cmake/haptic_wrist
)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/haptic_wristConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/haptic_wristConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/haptic_wristConfigVersion.cmake
  DESTINATION lib/cmake/haptic_wrist
)

add_subdirectory(programs)
