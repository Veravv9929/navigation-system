cmake_minimum_required(VERSION 3.14)
project(NavigationSystem VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall -Wextra")

#目录结构
include_directories(include)

#查找GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

#查找tinyxml2
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED tinyxml2)

#核心库
add_library(nav_core
	src/core/graph.cpp
	src/core/map_loader.cpp
	src/core/spatial_index.cpp
)

target_include_directories(nav_core PRIVATE ${TINYXML2_INCLUDE_DIRS})
target_link_libraries(nav_core ${TINYXML2_LIBRARIES})

#CLI可执行文件
add_executable(nav_cli src/main_cli.cpp)
target_link_libraries(nav_cli nav_core)

#测试
enable_testing()

add_executable(test_graph tests/test_graph.cpp)
target_link_libraries(test_graph nav_core ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)
target_include_directories(test_graph PRIVATE ${GTEST_INCLUDE_DIRS})


#注册到CTestf
add_test(NAME GraphTest COMMAND test_graph)
