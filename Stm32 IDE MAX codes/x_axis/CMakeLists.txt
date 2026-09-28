cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
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
set(CMAKE_PROJECT_NAME x_axis)

# Enable compile command
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create executable
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# =========================================================
# ADD USER SOURCE FILES
# =========================================================
target_sources(${CMAKE_PROJECT_NAME} PRIVATE

    Core/Src/encoder.c

)

# =========================================================
# INCLUDE PATHS
# =========================================================
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE

    Core/Inc

)

# =========================================================
# PROJECT DEFINITIONS
# =========================================================
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE

)

# Remove wrong libob.a dependency
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# =========================================================
# FLOATING-POINT PRINTF
# =========================================================
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -u _printf_float
)

# =========================================================
# LINK LIBRARIES
# =========================================================
target_link_libraries(${CMAKE_PROJECT_NAME}

    stm32cubemx

)