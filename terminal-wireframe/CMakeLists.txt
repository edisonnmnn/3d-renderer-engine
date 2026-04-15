cmake_minimum_required(VERSION 3.15)
project(wireframe_terminal)

set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang++)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(wireframe
    src/main.cpp
)

target_include_directories(wireframe PRIVATE src)
