macro(project_add_test TESTNAME)
    set(TEST_TARGET test_${TESTNAME})

    add_executable(${TEST_TARGET} ${ARGN})
    target_link_libraries(${TEST_TARGET} gtest gtest_main ${PROJECT_ALIAS})

    add_test(NAME ${TESTNAME} COMMAND ${TEST_TARGET})

endmacro(project_add_test)

project_add_test(differential_drive_model test_model.cpp)
project_add_test(genetic_algorithm test_ga_core.cpp test_ga_op.cpp)
project_add_test(single_nmpc_loop test_mono.cpp)

add_custom_target(run_tests ALL
    DEPENDS ${UNIT_TEST_TARGETS}
)

add_custom_command(
    TARGET run_tests
    COMMENT "Run unit tests"
    POST_BUILD
        COMMAND ctest
        ARGS --output-on-failure
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)