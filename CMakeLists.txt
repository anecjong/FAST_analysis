cmake_minimum_required(VERSION 3.10)
project(FAST_ANALYSIS
        LANGUAGES CXX
        )

set(CMAKE_CXX_STANDARD 14)
# 변수에 값을 넣어주기

set(CMAKE_CXX_STANDARD_REQUIRED ON)
# C++ 17, 20 등의 기능이 사용되면 기능 제한

add_subdirectory(modules)

add_executable(FAST_ANALYSIS exec/main.cpp)

target_link_libraries(FAST_ANALYSIS fast_classes)
