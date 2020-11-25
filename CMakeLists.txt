cmake_minimum_required(VERSION 3.18)

project(ga_nmpc_tuning CXX)
set(PROJECT_ALIAS "gnt")

option(BUILD_TESTS        "Build the tests"          ON)
option(BUILD_BENCHMARK    "Build benchmark binaries" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wshadow -fPIC)
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

set(CMAKE_BINARY_DIR        ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH  ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH     ${CMAKE_BINARY_DIR}/lib)

# ---------------------------------------------------------------------------------------
# Requirements
# ---------------------------------------------------------------------------------------
find_package(PkgConfig                         REQUIRED)
find_package(yaml-cpp    PATHS  lib/yaml-cpp   REQUIRED)
find_package(jsoncpp     PATHS  lib/jsoncpp    REQUIRED)
find_package(Eigen3 3.3  PATHS  lib/Eigen-3.3  REQUIRED)

pkg_check_modules(libCPPAD  REQUIRED  cppad)
pkg_check_modules(libIPOPT  REQUIRED  ipopt)

include_directories(
    include
)

link_libraries(
    ${libIPOPT_LIBRARIES}
    ${libCPPAD_LIBRARIES}
    yaml-cpp
    jsoncpp
)

set(PRECOMPILED_HEADER include/pcheaders.h)

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

add_library(${PROJECT_ALIAS} ${SOURCE_FILES})
target_precompile_headers(${PROJECT_ALIAS} PRIVATE ${PRECOMPILED_HEADER})

macro(project_add_target TARGETNAME)
    add_executable(${TARGETNAME} ${ARGN})
    target_link_libraries(${TARGETNAME} ${PROJECT_ALIAS})
    target_precompile_headers(${TARGETNAME} REUSE_FROM ${PROJECT_ALIAS})
endmacro(project_add_target)

project_add_target(mpc_mono src/mpc_mono.cpp)
project_add_target(hone_weights src/hone_weights.cpp)

# ---------------------------------------------------------------------------------------
# Testing
# ---------------------------------------------------------------------------------------
if(BUILD_TESTS)
    enable_testing()

    mark_as_advanced(
        BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS
        gmock_build_tests gtest_build_samples gtest_build_tests
        gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
    )
    add_subdirectory(lib/g-test)
    add_subdirectory(tests)
endif()

# pcheaders
# 472K	mpc_mono
# 540K	hone_weights
# real	0m29.671s
# user	0m27.856s
# sys	0m1.723s

# ---------------------------------------------------------------------------------------
# Benchmarking
# ---------------------------------------------------------------------------------------
if(BUILD_BENCHMARK)
    if(NOT BUILD_TESTS)
        mark_as_advanced(
            BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS
            gmock_build_tests gtest_build_samples gtest_build_tests
            gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
        )
        add_subdirectory(lib/g-test)
    endif()
    add_subdirectory(lib/g-benchmark)
    add_subdirectory(benchmark)
endif()
