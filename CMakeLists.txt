cmake_minimum_required(VERSION 3.16)
project(lacam2-project CXX)

add_subdirectory(./lacam2)
add_subdirectory(./third_party/argparse)
add_subdirectory(./third_party/googletest)

add_executable(main main.cpp)
target_compile_features(main PUBLIC cxx_std_17)
target_link_libraries(main lacam2 argparse)

# test
set(TEST_MAIN_FUNC ./third_party/googletest/googletest/src/gtest_main.cc)
file(GLOB TEST_FILES "./tests/test_*.cpp")
foreach(file ${TEST_FILES})
  string(REGEX MATCH "test\_[^\.]+" name "${file}")
  add_executable(${name} ${file} ${TEST_MAIN_FUNC})
  target_link_libraries(${name} lacam2 gtest)
endforeach()
add_executable(test_all ${TEST_MAIN_FUNC} ${TEST_FILES})
target_link_libraries(test_all lacam2 gtest)
