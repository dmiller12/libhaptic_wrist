
add_executable(
  run_tests
  test_kinematics.cpp
)

# Link the test executable against your library and its dependencies
target_link_libraries(
  run_tests
  PRIVATE
  haptic_wrist
  GTest::gtest_main
)

include(GoogleTest)

gtest_discover_tests(run_tests)
