cmake_minimum_required(VERSION 3.10.0)

project(drone-simulation
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB SOURCES CONFIGURE_DEPENDS
    "src/*.cpp"
)

add_executable(drone-simulation ${SOURCES}
    include/IMU.hpp
    src/IMU.cpp)

target_include_directories(drone-simulation PRIVATE
    include
)
