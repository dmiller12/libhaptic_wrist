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

add_executable(simple_commands simple_commands.cpp)
target_include_directories(simple_commands PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(simple_commands 
    haptic_wrist
    pthread
)

add_executable(joint_move joint_move.cpp)
target_include_directories(joint_move PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(joint_move 
    haptic_wrist
    pthread
)

add_executable(handle_commands handle_commands.cpp)
target_include_directories(handle_commands PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(handle_commands 
    haptic_wrist
    pthread
)
