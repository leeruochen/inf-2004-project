# robocar firmware: Pico W (RP2040) + micro T-Kernel 3.0, Pico SDK 1.5.1.
#
# RP2040 only. Core 1 stays unused; no SMP.
# Integration approach (a): the SDK owns crt0, boot2, clocks, linker script and
# vector table. The kernel is the static library `mtk3` (cmake/mtk3.cmake).
#
# Configure (see docs/DEBUG.md for the full Windows command lines):
#   cmake -S . -B build/uart0 -G Ninja -DCMAKE_BUILD_TYPE=Debug -DDBG_BACKEND=UART0
#   cmake -S . -B build/usb   -G Ninja -DCMAKE_BUILD_TYPE=Debug -DDBG_BACKEND=USB

cmake_minimum_required(VERSION 3.13)

# Board must be set before the SDK is imported.
set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(robocar C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)          # -std=gnu11
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

pico_sdk_init()

# --- Options -----------------------------------------------------------------
# Cache defaults below are seeded on the FIRST configure of a build directory
# only. To change a default (build type, DBG_DEFAULT), use a fresh build dir or
# pass the option itself (-DDBG_ENABLE=..., -DDBG_BACKEND=...).

# DBG_ENABLE: ON for debug builds, OFF for release builds (Release, MinSizeRel).
# pico_sdk_init() has already defaulted an empty CMAKE_BUILD_TYPE to Release.
if(CMAKE_BUILD_TYPE MATCHES "^(Release|MinSizeRel)$")
    set(_dbg_enable_default OFF)
else()
    set(_dbg_enable_default ON)
endif()
option(DBG_ENABLE "Debug ring + debug-only drain task (default ON for debug, OFF for release)" ${_dbg_enable_default})

set(DBG_DEFAULT "UART0" CACHE STRING "Default debug console backend (UART0 or USB); seeds DBG_BACKEND on first configure")
set(DBG_BACKEND "${DBG_DEFAULT}" CACHE STRING "Debug console backend: UART0 or USB")
set_property(CACHE DBG_BACKEND PROPERTY STRINGS UART0 USB)
if(NOT DBG_BACKEND MATCHES "^(UART0|USB)$")
    message(FATAL_ERROR "DBG_BACKEND='${DBG_BACKEND}' is invalid; use UART0 or USB")
endif()

option(TEL_BACKEND_WIFI "WiFi/MQTT telemetry backend (cyw43 + lwIP)" OFF)
if(TEL_BACKEND_WIFI)
    message(FATAL_ERROR "TEL_BACKEND_WIFI=ON is not implemented in Phase 1")
endif()
# TEL_BACKEND_WIFI=OFF: nothing links cyw43 or lwIP.

message(STATUS "robocar: DBG_ENABLE=${DBG_ENABLE} DBG_BACKEND=${DBG_BACKEND} TEL_BACKEND_WIFI=${TEL_BACKEND_WIFI}")

include(cmake/warnings.cmake)
include(cmake/mtk3.cmake)

# --- Rule checks (cmake/check_rules.cmake) -----------------------------------
add_custom_target(check_rules ALL
    COMMAND "${CMAKE_COMMAND}" "-DRULES_ROOT=${PROJECT_SOURCE_DIR}"
            -P "${PROJECT_SOURCE_DIR}/cmake/check_rules.cmake"
    COMMENT "Project rule checks"
    VERBATIM)

add_custom_target(check_rules_selftest
    COMMAND "${CMAKE_COMMAND}" -P "${PROJECT_SOURCE_DIR}/cmake/check_rules_selftest.cmake"
    COMMENT "Rule check self-test"
    VERBATIM)

# --- Project-wide options for OUR code ---------------------------------------
add_library(robocar_options INTERFACE)
target_include_directories(robocar_options INTERFACE "${PROJECT_SOURCE_DIR}")
if(DBG_ENABLE)
    target_compile_definitions(robocar_options INTERFACE DBG_ENABLE=1)
else()
    target_compile_definitions(robocar_options INTERFACE DBG_ENABLE=0)
endif()
if(DBG_BACKEND STREQUAL "UART0")
    target_compile_definitions(robocar_options INTERFACE DBG_BACKEND_UART0=1)
else()
    target_compile_definitions(robocar_options INTERFACE DBG_BACKEND_USB=1)
endif()

# --- Config header check (pure macros, our warning flags) --------------------
add_library(config_check OBJECT "${PROJECT_SOURCE_DIR}/cmake/config_check.c")
target_link_libraries(config_check PRIVATE robocar_options)
set_target_properties(config_check PROPERTIES C_STANDARD 11 C_EXTENSIONS ON)
robocar_apply_warnings("${PROJECT_SOURCE_DIR}/cmake/config_check.c")

# --- Module source registration ----------------------------------------------
# Module sources live under src/<module>/ (e.g. src/rtos/port/port_kernel_internal.c).
# Each module dir may provide src/<module>/sources.cmake that appends ABSOLUTE
# paths (use "${CMAKE_CURRENT_LIST_DIR}/x.c"):
#   ROBOCAR_SOURCES         SDK-side files (may include pico/..., hardware/...)
#   ROBOCAR_KERNEL_SOURCES  kernel-side files (include <tk/tkernel.h>)
# A file is in exactly one list. No globbing of sources.
set(ROBOCAR_SRC_DIR "${PROJECT_SOURCE_DIR}/src")
set(ROBOCAR_MODULE_DIRS common rtos debug motion vc safety line barcode obstacle imu telemetry app)

# Fail on any sources.cmake that would be silently ignored: only
# src/<listed module>/sources.cmake is registered.
# ProjectTemplate/ (course template, tracked as reference only) is never
# scanned or built: its matches are dropped before the check.
set(ROBOCAR_NEVER_SCAN_REGEX "^ProjectTemplate/")
set(_registered "")
foreach(_dir IN LISTS ROBOCAR_MODULE_DIRS)
    list(APPEND _registered "${ROBOCAR_SRC_DIR}/${_dir}/sources.cmake")
endforeach()
file(GLOB _found_top "${PROJECT_SOURCE_DIR}/*/sources.cmake")
set(_found_src "")
if(IS_DIRECTORY "${ROBOCAR_SRC_DIR}")
    file(GLOB_RECURSE _found_src "${ROBOCAR_SRC_DIR}/sources.cmake")
endif()
foreach(_f IN LISTS _found_top _found_src)
    file(RELATIVE_PATH _rel "${PROJECT_SOURCE_DIR}" "${_f}")
    if(_rel MATCHES "${ROBOCAR_NEVER_SCAN_REGEX}")
        continue()
    endif()
    list(FIND _registered "${_f}" _idx)
    if(_idx EQUAL -1)
        message(FATAL_ERROR "robocar: ${_rel} is not registered. Module sources.cmake files "
                            "must be src/<module>/sources.cmake with <module> in ROBOCAR_MODULE_DIRS.")
    endif()
endforeach()

set(ROBOCAR_SOURCES "")
set(ROBOCAR_KERNEL_SOURCES "")
foreach(_dir IN LISTS ROBOCAR_MODULE_DIRS)
    if(EXISTS "${ROBOCAR_SRC_DIR}/${_dir}/sources.cmake")
        include("${ROBOCAR_SRC_DIR}/${_dir}/sources.cmake")
    endif()
endforeach()

if(NOT ROBOCAR_SOURCES)
    message(STATUS "robocar: no module sources yet (Batch A); firmware target skipped")
else()
    if(ROBOCAR_KERNEL_SOURCES)
        # OBJECT library: its objects always land on the link line, which avoids
        # static-archive ordering cycles between the port and libmtk3.a.
        add_library(robocar_kside OBJECT ${ROBOCAR_KERNEL_SOURCES})
        target_link_libraries(robocar_kside PRIVATE mtk3_headers robocar_options)
        set_target_properties(robocar_kside PROPERTIES C_STANDARD 11 C_EXTENSIONS ON)
        robocar_apply_warnings(${ROBOCAR_KERNEL_SOURCES})
    endif()

    add_executable(robocar ${ROBOCAR_SOURCES})
    robocar_apply_warnings(${ROBOCAR_SOURCES})
    target_link_libraries(robocar PRIVATE
        robocar_options
        mtk3
        pico_stdlib
        hardware_pwm
        hardware_adc
        hardware_i2c
        hardware_gpio
        hardware_irq
        hardware_uart)
    if(TARGET robocar_kside)
        target_link_libraries(robocar PRIVATE robocar_kside)
    endif()

    pico_set_linker_script(robocar "${PROJECT_SOURCE_DIR}/cmake/memmap_mtk3.ld")

    if(DBG_BACKEND STREQUAL "UART0")
        pico_enable_stdio_uart(robocar 1)
        pico_enable_stdio_usb(robocar 0)
    else()
        pico_enable_stdio_uart(robocar 0)
        pico_enable_stdio_usb(robocar 1)
    endif()

    pico_add_extra_outputs(robocar)
    add_dependencies(robocar check_rules)
endif()

# --- Host unit tests (Batch D) -----------------------------------------------
# Host tests are configured standalone with a host compiler:
#   cmake -S tests -B build/host -G Ninja
# tests/CMakeLists.txt must return() when CMAKE_CROSSCOMPILING, and must not
# glob tests/rules_fixtures/ (rule-check fixtures, never compiled).
if(EXISTS "${PROJECT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
