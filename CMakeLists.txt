cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME 26_SuperCap_WirelessR_V2)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# 开启链接时优化 优化flash占用
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
file(GLOB_RECURSE SUPERCAP_SOURCES "User/SuperCap/*.c" "User/SuperCap/*.cpp")
file(GLOB_RECURSE WIRELESSR_SOURCES "User/WirelessR/*.c" "User/WirelessR/*.cpp")
file(GLOB GLOB_SOURCES "User/*.c" "User/*.cpp")
add_executable(${CMAKE_PROJECT_NAME}
        ${SUPERCAP_SOURCES}
        ${WIRELESSR_SOURCES}
        ${GLOB_SOURCES}
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        "/User/WirelessR/Inc"
        "/User/SuperCap"
        "/User/SuperCap/bmp"
        "/User/SuperCap/lcd"
        "/User/SuperCap/math_tool"
        "/User/SuperCap/music"
        "/User/SuperCap/task_manager"
        "/User/SuperCap/ws2812"
        "/User"
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
