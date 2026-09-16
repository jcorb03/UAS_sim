cmake_minimum_required(VERSION 3.20)

project(UAS_Sim
    VERSION 1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options("$<$<CONFIG:Debug>:/Zi>")
endif()

include(FetchContent)

FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
)

FetchContent_Declare(
    Eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 5.0.1
)

FetchContent_MakeAvailable(googletest Eigen)

# Main executable
add_executable(UAS_Sim
    src/main.cpp
    "src/dynamics/dynamics.cpp"
    src/sensors/GPSSensor.cpp
    src/estimation/UASEstimator.cpp
    src/control/Guidance.cpp
    src/Simulation.cpp 
    src/data/make_csv.cpp "src/UAS/UAS.cpp")

target_include_directories(UAS_Sim
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(UAS_Sim
    PRIVATE
        Eigen3::Eigen
)

# Tests
enable_testing()

add_executable(UAS_tests
    tests/dynamics/test_UAS.cpp
    tests/sensors/test_GPSSensor.cpp
    "src/dynamics/dynamics.cpp"
    src/sensors/GPSSensor.cpp
    src/estimation/UASEstimator.cpp
    src/control/Guidance.cpp
    src/Simulation.cpp
    tests/control/test_guidance.cpp 
    tests/estimation/test_GPS_kalman.cpp 
    tests/test_simulation01.cpp 
    src/data/make_csv.cpp "src/UAS/UAS.cpp")

target_include_directories(UAS_tests
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(UAS_tests
    PRIVATE
        GTest::gtest_main
        Eigen3::Eigen
)

include(GoogleTest)

gtest_discover_tests(UAS_tests)