cmake_minimum_required(VERSION 3.20)

project (EKF_NAV_INS)

# 1. 使用C++17标准
set(CMAKE_CXX_STANDARD 17)
# 2. 强制要求该标准（若编译器不支持，CMake会报错，避免隐式降级）
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 3. 禁用编译器扩展（可选，推荐：确保代码兼容标准C++，不依赖GCC/Clang特有语法）
set(CMAKE_CXX_EXTENSIONS OFF)

# 全局编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")
# 全局链接选项
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")

set(SOURCES
    ./src/algorithms/ekf_nav_ins/ekfNavINS.cpp
    ./src/drivers/gnss/cpp/gps_parser.cpp
    ./src/drivers/imu/cpp/imu_parser.cpp
    ./src/app/main_ekf_nav_ins.cpp
    ./src/utils/util.c
)

add_executable(EKF_NAV_INS ${SOURCES})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

target_include_directories(EKF_NAV_INS
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/src/algorithms/ekf_nav_ins/
        ${PROJECT_SOURCE_DIR}/src/drivers/gnss/cpp/
        ${PROJECT_SOURCE_DIR}/src/drivers/imu/cpp/
        ${PROJECT_SOURCE_DIR}/src/utils/
)
