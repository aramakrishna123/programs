cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# =========================================================
# COMPILER SETTINGS
# =========================================================

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# =========================================================
# BUILD TYPE
# =========================================================

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()


# =========================================================
# PROJECT NAME
# =========================================================

set(CMAKE_PROJECT_NAME x_axis_dtech)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)


# =========================================================
# PROJECT
# =========================================================

project(${CMAKE_PROJECT_NAME})

message("Build type: " ${CMAKE_BUILD_TYPE})


# =========================================================
# ENABLE C AND ASM
# =========================================================

enable_language(C ASM)


# =========================================================
# CREATE EXECUTABLE
# =========================================================

add_executable(${CMAKE_PROJECT_NAME})


# =========================================================
# STM32CUBEMX GENERATED SOURCES
# =========================================================

add_subdirectory(cmake/stm32cubemx)


# =========================================================
# LINK DIRECTORIES
# =========================================================

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE

    # Add user defined library search paths here if required

)


# =========================================================
# USER SOURCE FILES
# =========================================================

target_sources(${CMAKE_PROJECT_NAME} PRIVATE

    Core/Src/encoder.c
    

)


# =========================================================
# INCLUDE DIRECTORIES
# =========================================================

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE

    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc

)


# =========================================================
# PROJECT DEFINITIONS
# =========================================================

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE

    # Add user defined symbols here if required

)


# =========================================================
# REMOVE WRONG libob.a DEPENDENCY
# =========================================================

list(REMOVE_ITEM
    CMAKE_C_IMPLICIT_LINK_LIBRARIES
    ob
)


# =========================================================
# LINK LIBRARIES
# =========================================================

target_link_libraries(${CMAKE_PROJECT_NAME}

    stm32cubemx

    # Add user defined libraries here if required

)