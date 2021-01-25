cmake_minimum_required(VERSION 3.13)

project(ga_nmpc_tuning
    VERSION 0.0.1
    LANGUAGES CXX
)
set(PROJECT_ALIAS "gnt")

option(BUILD_TESTS        "Build the tests"          OFF)
option(BUILD_BENCHMARK    "Build benchmark binaries" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

find_package(PkgConfig REQUIRED)

pkg_check_modules(CPPAD  REQUIRED  cppad)
pkg_check_modules(IPOPT  REQUIRED  ipopt)

find_package(jsoncpp REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
    include
)

link_libraries(
    ${IPOPT_LIBRARIES}
    ${CPPAD_LIBRARIES}
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
add_library(${PROJECT_ALIAS} STATIC ${SOURCE_FILES})

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
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# ---------------------------------------------------------------------------------------
# Benchmarking
# ---------------------------------------------------------------------------------------
if(BUILD_BENCHMARK)
    add_subdirectory(benchmark)
endif()
