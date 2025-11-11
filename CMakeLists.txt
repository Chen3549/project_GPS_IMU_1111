cmake_minimum_required(VERSION 3.12)

SET(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/build/)
project(Positioning)

add_executable(Positioning src/user/main.c)
