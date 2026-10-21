cmake_minimum_required(VERSION 3.20)

project(DoublePendulumLab LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(SFML COMPONENTS Graphics Window System CONFIG REQUIRED)

add_executable(DoublePendulumLab
    main.cpp
)

target_link_libraries(DoublePendulumLab PRIVATE
    SFML::Graphics
    SFML::Window
    SFML::System
)

