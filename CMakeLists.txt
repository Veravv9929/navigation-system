cmake_minimum_required(VERSION 3.14)
project(NavigationSystem VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -02 -Wall -Wextra")

//目录结构
include_directories(include)

//核心库
add_library(nav_core
	src/core/graph.cpp
	src/core/map_loader.cpp
	src/core/spatial_index.cpp
)

//可执行文件
add_executable(nav_cli src/main_cli.cpp)
target_link_libraries(nav_cli nav_core)

//测试
enable_testing()
find_package(GTest REQUIRED)
add_xecutable(test_graph tests/test_graph.cpp)
target_link_libraries(test_graph nav_core GTest::gtest_main)
