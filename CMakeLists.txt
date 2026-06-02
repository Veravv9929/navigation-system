cmake_minimum_required(VERSION 3.14)
project(NavigationSystem VERSION 1.0.0 LANGUAGES CXX C)

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
    src/core/routing.cpp
    src/core/contraction_hierarchies.cpp
    src/core/algorithm_router.cpp
)

target_include_directories(nav_core PRIVATE ${TINYXML2_INCLUDE_DIRS})
target_link_libraries(nav_core ${TINYXML2_LIBRARIES})

#CLI可执行文件
add_executable(nav_cli src/main_cli.cpp)
target_link_libraries(nav_cli nav_core)

#=============================C++ JSON引擎==================================
add_executable(nav_engine src/cli/json_engine.cpp)
target_link_libraries(nav_engine nav_core)
target_include_directories(nav_engine PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

# ========== mongoose HTTP 库 ==========
add_library(mongoose third_party/mongoose/mongoose.c)
target_compile_definitions(mongoose PRIVATE MG_ENABLE_LINES=1)
set_source_files_properties(third_party/mongoose/mongoose.c PROPERTIES LANGUAGE C)
set_target_properties(mongoose PROPERTIES LINKER_LANGUAGE C)

# ========== HTTP 服务器 ==========
add_executable(nav_server
    src/server/http_server.cpp
)
target_link_libraries(nav_server
    mongoose
    nav_core
)
target_include_directories(nav_server PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mongoose
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party  # ← 加这行，让 json.hpp 能找到
)

#测试(graph)
enable_testing()

add_executable(test_graph tests/test_graph.cpp)
target_link_libraries(test_graph nav_core ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)
target_include_directories(test_graph PRIVATE ${GTEST_INCLUDE_DIRS})


#注册到CTestf
add_test(NAME GraphTest COMMAND test_graph)


#测试(algorithm)
add_executable(test_routing tests/test_routing.cpp)
target_link_libraries(test_routing nav_core ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)
target_include_directories(test_routing PRIVATE ${GTEST_INCLUDE_DIRS})
add_test(NAME RoutingTest COMMAND test_routing)


#路由性能测试
add_executable(benchmark_routing tests/benchmark_routing.cpp)
target_link_libraries(benchmark_routing nav_core ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)
target_include_directories(benchmark_routing PRIVATE ${GTEST_INCLUDE_DIRS})
add_test(NAME BenchmarkRouting COMMAND benchmark_routing)


#测试CH小图
add_executable(test_ch_simple tests/test_ch_simple.cpp)
target_link_libraries(test_ch_simple nav_core ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)
target_include_directories(test_ch_simple PRIVATE ${GTEST_INCLUDE_DIRS})
add_test(NAME TestChSimple COMMAND test_ch_simple)


#测试CH大图
add_executable(test_ch_stall tests/test_ch_stall.cpp)
target_link_libraries(test_ch_stall nav_core ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)
target_include_directories(test_ch_stall PRIVATE ${GTEST_INCLUDE_DIRS})
add_test(NAME TestChStall COMMAND test_ch_stall)

#测试algorithm集合
add_executable(test_algorithm_router tests/test_algorithm_router.cpp)
target_link_libraries(test_algorithm_router nav_core ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)
target_include_directories(test_algorithm_router PRIVATE ${GTEST_INCLUDE_DIRS})
add_test(NAME TestAlgorithmRouter COMMAND test_algorithm_router)
