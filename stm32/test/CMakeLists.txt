cmake_minimum_required(VERSION 2.8.8)
project(mylittleproject)

add_library(logic STATIC logic.c)

add_executable(mylittleprogram main.c)

target_link_libraries(mylittleprogram logic)
# set(CMAKE_CXX_STANDARD 14)

# include(CTest)
# set(INSTALL_GTEST OFF)

if(BUILD_TESTING)
enable_testing()
  add_subdirectory(googletest)
  add_executable(hellotest tests.cpp)
  target_link_libraries(hellotest gtest)
  add_test(gtest_test hellotest)
  # ... CMake code to create tests ...
  add_test(first_test mylittleprogram 1 1 1 1 1 1)
  set(MULTIPLY_TIMEOUT 1)
  set_tests_properties(first_test PROPERTIES TIMEOUT ${MULTIPLY_TIMEOUT})
  set_tests_properties(first_test PROPERTIES 
  PASS_REGULAR_EXPRESSION "SUCCESS" 
  FAIL_REGULAR_EXPRESSION "Error!")
endif()


