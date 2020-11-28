cmake_minimum_required(VERSION 3.18)

project(ga_nmpc_tuning CXX)
set(PROJECT_ALIAS "gnt")

option(BUILD_TESTS        "Build the tests"          ON)
option(BUILD_BENCHMARK    "Build benchmark binaries" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-w -fPIC)
if(NOT BUILD_BENCHMARK)
    add_compile_definitions(NBENCHMARK)
endif()

# ---------------------------------------------------------------------------------------
# Set default build to release
# ---------------------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()
message(STATUS "Generating with build type: ${CMAKE_BUILD_TYPE}")

file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/data)

# ---------------------------------------------------------------------------------------
# Requirements
# ---------------------------------------------------------------------------------------
add_subdirectory(lib/yaml-cpp)
add_subdirectory(lib/jsoncpp)
add_subdirectory(lib/Eigen-3.3)

find_package(PkgConfig REQUIRED)
pkg_check_modules(libCPPAD  REQUIRED  cppad)
pkg_check_modules(libIPOPT  REQUIRED  ipopt)

include_directories(
    lib/Eigen-3.3
    lib/yaml-cpp/include
    lib/jsoncpp/include
    include
)

# Jsoncpp authors have explicitly set output directories
# Removing this will cause linking issues for jsoncpp library
link_directories(${CMAKE_BINARY_DIR}/lib)

link_libraries(
    ${libIPOPT_LIBRARIES}
    ${libCPPAD_LIBRARIES}
    yaml-cpp
    jsoncpp
)

# Library source files
set(SOURCE_FILES
    include/utils/config_handler.hpp
    include/utils/json_logger.hpp
    include/utils/progress_bar.hpp
    src/mpc_lib/mpc.cpp
    src/model/differential_drive.cpp
    src/model/base_organism.cpp
    src/genetic_algorithm/core.cpp 
    src/genetic_algorithm/operators.cpp
    src/genetic_algorithm/organism.cpp
    src/genetic_algorithm/fitness.cpp
    src/genetic_algorithm/population.cpp
)

# Project library
add_library(${PROJECT_ALIAS} ${SOURCE_FILES})

# ---------------------------------------------------------------------------------------
# Project targets
# ---------------------------------------------------------------------------------------
macro(project_add_target TARGETNAME)
    add_executable(${TARGETNAME} ${ARGN})
    target_link_libraries(${TARGETNAME} ${PROJECT_ALIAS})
endmacro(project_add_target)

project_add_target(mpc_mono src/mpc_mono.cpp)
project_add_target(hone_weights src/hone_weights.cpp)

# ---------------------------------------------------------------------------------------
# Testing
# ---------------------------------------------------------------------------------------
macro(add_gtest)
    mark_as_advanced(
        BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS
        gmock_build_tests gtest_build_samples gtest_build_tests
        gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
    )
    add_subdirectory(lib/g-test)
endmacro(add_gtest)

if(BUILD_TESTS)
    enable_testing()
    add_gtest()
    add_subdirectory(tests)
endif()

# ---------------------------------------------------------------------------------------
# Benchmarking
# ---------------------------------------------------------------------------------------
if(BUILD_BENCHMARK)
    # google-benchmark requires googletest
    if(NOT BUILD_TESTS)
        add_gtest()
    endif()
    add_subdirectory(lib/g-benchmark)
    add_subdirectory(benchmark)
endif()
