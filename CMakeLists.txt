# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_CXX_ENABLE_EXCEPTIONS 1)
set(PICO_CXX_ENABLE_RTTI 1)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Robot-Arm C CXX ASM)

include(FreeRTOS_Kernel_import.cmake)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(Robot-Arm main.cpp Robot-Arm.cpp Communication.cpp)

pico_set_program_name(Robot-Arm "Robot-Arm")
pico_set_program_version(Robot-Arm "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Robot-Arm 0)
pico_enable_stdio_usb(Robot-Arm 1)
target_compile_definitions(Robot-Arm PRIVATE PICO_ENTER_USB_BOOT_ON_EXIT=1)

# Add the standard library to the build
target_link_libraries(Robot-Arm
        pico_stdlib hardware_pwm)

# Add the standard include files to the build
target_include_directories(Robot-Arm PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Manually added
# FILE(GLOB_RECURSE EIGEN_INC ${PROJECT_SOURCE_DIR}/eigen/Eigen/)
# add_subdirectory(${PROJECT_SOURCE_DIR}/eigen)
# aux_source_directory(${PROJECT_SOURCE_DIR}/eigen/Eigen/src/ EIGEN_SRCS)
# add_library(Eigen3 ${EIGEN_INC})
# set_target_properties(Eigen3 PROPERTIES LINKER_LANGUAGE CXX)
# target_include_directories(${EIGEN_SRCS})

# FILE(GLOB_RECURSE QUIK_INC ${PROJECT_SOURCE_DIR}/quik/*.hpp)
# add_subdirectory(${PROJECT_SOURCE_DIR}/quik)
# include_directories(${PROJECT_SOURCE_DIR}/eigen)
# include_directories(${PROJECT_SOURCE_DIR}/quik)
FILE(GLOB NANOPB_INC ${PROJECT_SOURCE_DIR}/nanopb/*.h)
FILE(GLOB NANOPB_SRC ${PROJECT_SOURCE_DIR}/nanopb/*.c)
add_library(quik ${PROJECT_SOURCE_DIR}/quik-embedded/src/geometry.cpp ${QUIK_INC})
add_library(nanopb ${NANOPB_SRC} ${NANOPB_INC})
# find_package(nanopb CONFIG REQUIRED)
# target_link_libraries(Robot-Arm ${PROJECT_SOURCE_DIR}/nanopb)
target_include_directories(quik PRIVATE ${PROJECT_SOURCE_DIR}/quik-embedded/include ${PROJECT_SOURCE_DIR}/eigen/)
target_include_directories(Robot-Arm PRIVATE ${PROJECT_SOURCE_DIR}/quik-embedded/include ${PROJECT_SOURCE_DIR}/eigen/ ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/nanopb/)
# find_package(quik)
# find_package(Eigen3)

# Add any user requested libraries
target_link_libraries(Robot-Arm 
        quik
        # Eigen3
        )

# FreeRTOS stuff
# set(FREERTOS_KERNEL_PATH ${PROJECT_SOURCE_DIR}/FreeRTOS-Kernel/)
# include(FreeRTOS_Kernel_import.cmake)

# Add the freertos_config for FreeRTOS-Kernel
# add_library(freertos_config INTERFACE)

# target_include_directories(freertos_config
#     INTERFACE
#     ${PROJECT_SOURCE_DIR}
# )

# if (DEFINED FREERTOS_SMP_EXAMPLE AND FREERTOS_SMP_EXAMPLE STREQUAL "1")
#     message(STATUS "Build FreeRTOS SMP example")
#     # Adding the following configurations to build SMP template port
#     add_compile_options( -DconfigNUMBER_OF_CORES=2 -DconfigUSE_PASSIVE_IDLE_HOOK=0 )
# endif()

# Select the heap port.  values between 1-4 will pick a heap.
# set(FREERTOS_HEAP "4" CACHE STRING "" FORCE)

# Select the native compile PORT
# set(FREERTOS_PORT "ThirdParty_GCC_RP2350_ARM_NTZ" CACHE STRING "" FORCE)

# target_compile_definitions(freertos_config INTERFACE ${definitions})
# target_compile_options(freertos_config INTERFACE ${options})

# Adding the FreeRTOS-Kernel subdirectory
# add_subdirectory(${FREERTOS_KERNEL_PATH} FreeRTOS-Kernel)

# target_include_directories(Robot-Arm PRIVATE ${FREERTOS_KERNEL_PATH}/include)

########################################################################
# Overall Compile Options
# Note the compile option strategy is to error on everything and then
# Per library opt-out of things that are warnings/errors.
# This ensures that no matter what strategy for compilation you take, the
# builds will still occur.
#
# Only tested with GNU and Clang.
# Other options are https://cmake.org/cmake/help/latest/variable/CMAKE_LANG_COMPILER_ID.html#variable:CMAKE_%3CLANG%3E_COMPILER_ID
# Naming of compilers translation map:
#
#   FreeRTOS    | CMake
#   -------------------
#   CCS         | ?TBD?
#   GCC         | GNU, Clang, *Clang Others?
#   IAR         | IAR
#   Keil        | ARMCC
#   MSVC        | MSVC # Note only for MinGW?
#   Renesas     | ?TBD?

target_compile_options(Robot-Arm PRIVATE
        ### Gnu/Clang C Options
        $<$<COMPILE_LANG_AND_ID:C,GNU>:-fdiagnostics-color=always>
        $<$<COMPILE_LANG_AND_ID:C,Clang>:-fcolor-diagnostics>

        $<$<COMPILE_LANG_AND_ID:C,Clang,GNU>:-Wall>
        $<$<COMPILE_LANG_AND_ID:C,Clang,GNU>:-Wextra>
        # $<$<COMPILE_LANG_AND_ID:C,Clang,GNU>:-Werror>
        $<$<COMPILE_LANG_AND_ID:C,Clang>:-Weverything>
)

# target_include_directories(FreeRTOS-Kernel ${PROJECT_SOURCE_DIR})
# target_link_libraries(Robot-Arm freertos_kernel freertos_config)
target_link_libraries(Robot-Arm FreeRTOS-Kernel FreeRTOS-Kernel-Heap4)

# set_property(TARGET freertos_kernel PROPERTY C_STANDARD 99)

pico_add_extra_outputs(Robot-Arm)
