add_executable(bm_nmpc bm_nmpc_loop.cpp)
target_link_libraries(bm_nmpc benchmark::benchmark ${PROJECT_ALIAS})
target_precompile_headers(bm_nmpc REUSE_FROM ${PROJECT_ALIAS})
