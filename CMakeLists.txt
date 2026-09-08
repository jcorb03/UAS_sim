cmake_minimum_required(VERSION 3.20)

project(UAS_Sim
    VERSION 1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(UAS_Sim
    src/main.cpp
    src/dynamics/UAS.cpp
)

target_include_directories(UAS_Sim
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

include(FetchContent)

FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
)

FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(UAS_tests
    tests/dynamics/test_UAS.cpp
    src/dynamics/UAS.cpp
)

target_include_directories(UAS_tests
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(UAS_tests
    PRIVATE
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(UAS_tests)