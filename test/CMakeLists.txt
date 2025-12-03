cmake_minimum_required(VERSION 3.5...3.30)
project(UNIT_TESTS)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(GTest REQUIRED)

set(TEST_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/unit/main_test.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/drivers/gnss/c/gps_parser.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/utils/util.c
)

add_executable(unit_tests ${TEST_SRCS})

target_link_libraries(unit_tests
  PRIVATE ${GTEST_LIBRARIES}
  PRIVATE gmock gmock_main
  $<$<PLATFORM_ID:Linux>:pthread>
)

target_include_directories(unit_tests
  PRIVATE ${GTEST_INCLUDE_DIRS}
  PRIVATE /usr/include/gmock
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src/drivers/gnss/c
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src/utils
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/unit
)

enable_testing()
add_test(
  NAME GpsParserTests
  COMMAND $<TARGET_FILE:unit_tests>
)
