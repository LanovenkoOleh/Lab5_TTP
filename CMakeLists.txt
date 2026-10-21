cmake_minimum_required(VERSION 3.25)

project(NBodySimulation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(SFML COMPONENTS Graphics Window System CONFIG REQUIRED)

add_executable(NBodySimulation main.cpp)

target_link_libraries(NBodySimulation PRIVATE
    SFML::Graphics
    SFML::Window
    SFML::System
)

file(COPY "${CMAKE_SOURCE_DIR}/resources"
     DESTINATION "${CMAKE_BINARY_DIR}")

set_property(TARGET NBodySimulation PROPERTY
    VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
)
