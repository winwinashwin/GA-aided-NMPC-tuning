cmake_minimum_required(VERSION 3.18)

project(ga_nmpc_tuning VERSION 0.0)
set(PROJECT_ALIAS "gnt")

option(PROJECT_TESTS "Build the tests"          OFF)
option(PROJECT_BM    "Build in benchmark mode"  OFF)

# ---------------------------------------------------------------------------------------
# Set default build to release
# ---------------------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()
message(STATUS "Generating with build type: ${CMAKE_BUILD_TYPE}")

# ---------------------------------------------------------------------------------------
# Compile time configurations
# ---------------------------------------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT PROJECT_BM)
    # Define `NMODE_BM` macro if not in bm mode. Else all console output will be suppressed.
    add_compile_options(-D NMODE_BM)
endif()

# ---------------------------------------------------------------------------------------
# Set up directories
# ---------------------------------------------------------------------------------------
set(CMAKE_BINARY_DIR        ${CMAKE_SOURCE_DIR}/_bin)
set(EXECUTABLE_OUTPUT_PATH  ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH     ${CMAKE_BINARY_DIR}/lib)

file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/data)

# ---------------------------------------------------------------------------------------
# Requirements
# ---------------------------------------------------------------------------------------
find_package(PkgConfig   REQUIRED)
find_package(yaml-cpp    REQUIRED)
find_package(jsoncpp     REQUIRED)
find_package(Eigen3 3.3  REQUIRED)

pkg_check_modules(libCPPAD  REQUIRED  cppad)
pkg_check_modules(libIPOPT  REQUIRED  ipopt)


set(LINK_DEPS
    yaml-cpp
    jsoncpp
    ${libIPOPT_LIBRARIES}
    ${libCPPAD_LIBRARIES}
    Eigen3::Eigen
)
set(PRECOMPILED_HEADER include/pcheaders.h)

set(LIB_SOURCES
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

# Update include path
include_directories("include")

# ---------------------------------------------------------------------------------------
# Project library
# ---------------------------------------------------------------------------------------
add_library(${PROJECT_ALIAS} ${LIB_SOURCES})
target_link_libraries(${PROJECT_ALIAS} ${LINK_DEPS})
set_target_properties(${PROJECT_ALIAS} PROPERTIES LINKER_LANGUAGE CXX)
target_precompile_headers(${PROJECT_ALIAS} PRIVATE ${PRECOMPILED_HEADER})

# ---------------------------------------------------------------------------------------
# Executables
# ---------------------------------------------------------------------------------------
add_executable(mpc_mono src/mpc_mono.cpp)
target_link_libraries(mpc_mono ${PROJECT_ALIAS})
target_precompile_headers(mpc_mono REUSE_FROM ${PROJECT_ALIAS})

add_executable(hone_weights src/hone_weights.cpp)
target_link_libraries(hone_weights ${PROJECT_ALIAS})
target_precompile_headers(hone_weights REUSE_FROM ${PROJECT_ALIAS})

# ---------------------------------------------------------------------------------------
# Unit tests
# ---------------------------------------------------------------------------------------
if(PROJECT_TESTS)
    enable_testing()
    add_subdirectory(lib/googletest)
    add_subdirectory(tests)
endif()
