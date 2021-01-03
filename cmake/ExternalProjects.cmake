include(FindGit)
find_package(Git)

if (NOT Git_FOUND)
	message(FATAL_ERROR "Git not found!")
endif ()

include(FetchContent)

FetchContent_Declare(jsoncpp
    GIT_REPOSITORY    https://github.com/ashwin5059198/jsoncpp.git
    GIT_TAG           master
)

FetchContent_Declare(yaml-cpp
    GIT_REPOSITORY    https://github.com/ashwin5059198/yaml-cpp.git
    GIT_TAG           master
)

FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.10.x
)

FetchContent_Declare(google-benchmark
    GIT_REPOSITORY https://github.com/google/benchmark.git
    GIT_TAG master # need master for benchmark::benchmark
)

FetchContent_MakeAvailable(
    jsoncpp
    yaml-cpp
    googletest
    google-benchmark
)