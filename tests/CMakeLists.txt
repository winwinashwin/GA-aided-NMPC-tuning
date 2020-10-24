set(TEST_SOURCES
    test_model.cpp
    test_ga_core.cpp
    test_ga_op.cpp
    test_mono.cpp
)
set(TEST_EXE ${PROJECT_ALIAS}_run_tests)

add_executable(${TEST_EXE} ${TEST_SOURCES})
target_link_libraries(${TEST_EXE} gtest gtest_main ${PROJECT_ALIAS})
target_precompile_headers(${TEST_EXE} REUSE_FROM ${PROJECT_ALIAS})
add_test(${TEST_EXE} run_tests)
