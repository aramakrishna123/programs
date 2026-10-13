cmake_minimum_required(VERSION 3.22)

project(x_axis_wireless C ASM)


# =========================================================
# MAIN EXECUTABLE
# =========================================================

add_executable(${CMAKE_PROJECT_NAME})


# =========================================================
# USER SOURCE FILES
# =========================================================

target_sources(${CMAKE_PROJECT_NAME} PRIVATE

    Core/Src/main.c

    Core/Src/encoder.c

    Core/Src/stm32f4xx_it.c

    Core/Src/stm32f4xx_hal_msp.c

    Core/Src/syscalls.c

    Core/Src/sysmem.c

    startup_stm32f446xx.s
)


# =========================================================
# INCLUDE DIRECTORIES
# =========================================================

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE

    Core/Inc
)


# =========================================================
# STM32CUBEMX GENERATED PROJECT
# =========================================================

add_subdirectory(
    cmake/stm32cubemx
)


# =========================================================
# LINK STM32 DRIVERS
# =========================================================

target_link_libraries(
    ${CMAKE_PROJECT_NAME}
    PRIVATE
    STM32_Drivers
)


# =========================================================
# CPU / COMPILE SETTINGS
# =========================================================

target_compile_options(
    ${CMAKE_PROJECT_NAME}
    PRIVATE

    -mcpu=cortex-m4

    -mfpu=fpv4-sp-d16

    -mfloat-abi=hard

    -Wall

    -fdata-sections

    -ffunction-sections

    -fstack-usage
)


# =========================================================
# LINKER OPTIONS
# =========================================================

target_link_options(
    ${CMAKE_PROJECT_NAME}
    PRIVATE

    -mcpu=cortex-m4

    -mfpu=fpv4-sp-d16

    -mfloat-abi=hard

    -u _printf_float

    
)


# =========================================================
# STM32 DEFINITIONS
# =========================================================

target_compile_definitions(
    ${CMAKE_PROJECT_NAME}
    PRIVATE

    STM32F446xx

    USE_HAL_DRIVER

    USE_NUCLEO_64
)