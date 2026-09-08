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
