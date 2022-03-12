cmake_minimum_required(VERSION 3.23)
# Add pico sdk
include(pico_sdk_import.cmake)

project(cs200 C CXX ASM)
set(CMAKE_C_STANDARD 11)

# Initialize the SDK
pico_sdk_init()

# Add qmc subdirectory
add_subdirectory(gy9250_i2c)
