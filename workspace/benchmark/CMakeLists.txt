macro(project_add_benchmark BMNAME)

    set(BM_TARGET bm_${BMNAME})
    add_executable(${BM_TARGET} ${ARGN})
    target_link_libraries(${BM_TARGET} benchmark ${PROJECT_ALIAS})

endmacro(project_add_benchmark)

project_add_benchmark(nmpc bm_nmpc_loop.cpp)