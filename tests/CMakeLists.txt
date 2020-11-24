enable_testing()

set(TEST_LIBRARIES
    gtest
    gtest_main
    ${PROJECT_ALIAS}
)

add_executable(test_model test_model.cpp)
target_link_libraries(test_model ${TEST_LIBRARIES})
target_precompile_headers(test_model REUSE_FROM ${PROJECT_ALIAS})
add_test(NAME T_models COMMAND test_model)

add_executable(test_ga test_ga_core.cpp test_ga_op.cpp)
target_link_libraries(test_ga ${TEST_LIBRARIES})
target_precompile_headers(test_ga REUSE_FROM ${PROJECT_ALIAS})
add_test(NAME T_genetic_algorithm COMMAND test_ga)

add_executable(test_mono test_mono.cpp)
target_link_libraries(test_mono ${TEST_LIBRARIES})
target_precompile_headers(test_mono REUSE_FROM ${PROJECT_ALIAS})
add_test(NAME T_MPC_loop COMMAND test_mono)
