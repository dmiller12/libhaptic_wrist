include_directories(../src)

if (BUILD_BARRETT)
  find_package(Barrett REQUIRED)
  include_directories(${BARRETT_INCLUDE_DIRS})
  link_directories(${BARRETT_LIBRARY_DIRS})
  add_definitions(${BARRETT_DEFINITIONS})

  add_executable(gravity_cal gravity_cal.cpp)
  target_link_libraries(gravity_cal haptic_wrist ${BARRETT_LIBRARIES} yaml-cpp::yaml-cpp) 

  add_executable(demo_grav_comp demo_grav_comp.cpp)
  target_link_libraries(demo_grav_comp haptic_wrist ${BARRETT_LIBRARIES}) 
endif()

add_executable(test_wrist test_wrist.cpp)
target_include_directories(test_wrist PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(test_wrist 
    haptic_wrist
    pthread
    yaml-cpp
)
