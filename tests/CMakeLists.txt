macro(project_add_test TESTNAME)
    set(TEST_TARGET test_${TESTNAME})

    add_executable(${TEST_TARGET} ${ARGN})
    target_link_libraries(${TEST_TARGET} gtest gtest_main ${PROJECT_ALIAS})

    add_test(NAME T_${TESTNAME} COMMAND ${TEST_TARGET})

endmacro(project_add_test)

project_add_test(model test_model.cpp)
project_add_test(ga test_ga_core.cpp test_ga_op.cpp)
project_add_test(mono test_mono.cpp)
