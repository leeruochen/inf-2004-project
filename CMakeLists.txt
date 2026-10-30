# robocar firmware: Pico W (RP2040) + micro T-Kernel 3.0, Pico SDK 1.5.1.
#
# RP2040 only. Core 1 stays unused; no SMP.
# Integration approach (a): the SDK owns crt0, boot2, clocks, linker script and
# vector table. The kernel is the static library `mtk3` (cmake/mtk3.cmake).
#
# Configure (see docs/DEBUG.md for the full Windows command lines):
#   cmake -S . -B build/usb -G Ninja -DCMAKE_BUILD_TYPE=Debug -DDBG_BACKEND=USB
# USB CDC is the only console backend; DBG_BACKEND=UART0 fails at configure.

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

# APP_TEST: every subsystemN_test() runs in a dedicated test task at
# TASK_PRI_TEST (11, below the debug drain). Results go through the dbg_ ring.
option(APP_TEST "On-target subsystem self-tests in a lowest-priority test task (needs DBG_ENABLE)" OFF)
if(APP_TEST AND NOT DBG_ENABLE)
    message(FATAL_ERROR "APP_TEST=ON needs DBG_ENABLE=ON: test results are printed through the dbg_ ring")
endif()

# USB CDC is the only console (Week 8 pin map v2, ARCHITECTURE.md D6). The
# Debug Probe is SWD only. UART0 would drive GP0 against the right encoder.
set(DBG_DEFAULT "USB" CACHE STRING "Default debug console backend (USB only); seeds DBG_BACKEND on first configure")
set(DBG_BACKEND "${DBG_DEFAULT}" CACHE STRING "Debug console backend: USB")
set_property(CACHE DBG_BACKEND PROPERTY STRINGS USB)
if(DBG_BACKEND STREQUAL "UART0")
    message(FATAL_ERROR "DBG_BACKEND=UART0 was removed (Week 8 pin map v2): GP0/GP1 are ENC_R_A/ENC_R_B "
                        "in config/pins.h. USB CDC is the only console (docs/DEBUG.md).")
endif()
if(NOT DBG_BACKEND STREQUAL "USB")
    message(FATAL_ERROR "DBG_BACKEND='${DBG_BACKEND}' is invalid; use USB")
endif()

option(TEL_BACKEND_WIFI "WiFi/MQTT telemetry backend (cyw43 + lwIP)" OFF)
if(TEL_BACKEND_WIFI)
    message(FATAL_ERROR "TEL_BACKEND_WIFI=ON is not implemented in Phase 1")
endif()
# TEL_BACKEND_WIFI=OFF: nothing links cyw43 or lwIP.

# Platform self-test build (Batch B stage B4b, batchB-plan B4): forced fault,
# fault record, CRC and stack checks.  Never in a normal build.
option(PLATFORM_SELFTEST "Platform self-test task (debug builds only; forces a HardFault once per power-up)" OFF)
if(PLATFORM_SELFTEST AND NOT DBG_ENABLE)
    message(FATAL_ERROR "PLATFORM_SELFTEST=ON needs DBG_ENABLE=ON (its results go through the dbg_ ring)")
endif()
if(PLATFORM_SELFTEST AND APP_TEST)
    message(FATAL_ERROR "PLATFORM_SELFTEST=ON and APP_TEST=ON cannot be combined (one extra task fits the budget, config/priorities.h)")
endif()

# Platform bring-up build (Batch B stage B5, batchB-plan O1): nine heartbeat
# tasks INSTEAD of the subsystems' tasks, for the USB CDC bring-up test
# (docs/DEBUG.md).  Never in a normal build.
option(PLATFORM_BRINGUP "Bring-up heartbeat tasks instead of the subsystems (debug builds only)" OFF)
if(PLATFORM_BRINGUP AND NOT DBG_ENABLE)
    message(FATAL_ERROR "PLATFORM_BRINGUP=ON needs DBG_ENABLE=ON (the heartbeats go through the dbg_ ring)")
endif()
if(PLATFORM_BRINGUP AND (APP_TEST OR PLATFORM_SELFTEST))
    message(FATAL_ERROR "PLATFORM_BRINGUP=ON cannot be combined with APP_TEST or PLATFORM_SELFTEST (the test measures the platform alone)")
endif()

message(STATUS "robocar: DBG_ENABLE=${DBG_ENABLE} DBG_BACKEND=${DBG_BACKEND} TEL_BACKEND_WIFI=${TEL_BACKEND_WIFI} APP_TEST=${APP_TEST} PLATFORM_SELFTEST=${PLATFORM_SELFTEST} PLATFORM_BRINGUP=${PLATFORM_BRINGUP}")

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
target_compile_definitions(robocar_options INTERFACE DBG_BACKEND_USB=1)
if(APP_TEST)
    target_compile_definitions(robocar_options INTERFACE APP_TEST=1)
else()
    target_compile_definitions(robocar_options INTERFACE APP_TEST=0)
endif()
if(PLATFORM_SELFTEST)
    target_compile_definitions(robocar_options INTERFACE PLATFORM_SELFTEST=1)
else()
    target_compile_definitions(robocar_options INTERFACE PLATFORM_SELFTEST=0)
endif()
if(PLATFORM_BRINGUP)
    target_compile_definitions(robocar_options INTERFACE PLATFORM_BRINGUP=1)
else()
    target_compile_definitions(robocar_options INTERFACE PLATFORM_BRINGUP=0)
endif()

# --- Config header check (pure macros, our warning flags) --------------------
add_library(config_check OBJECT "${PROJECT_SOURCE_DIR}/cmake/config_check.c")
target_link_libraries(config_check PRIVATE robocar_options)
set_target_properties(config_check PROPERTIES C_STANDARD 11 C_EXTENSIONS ON)
robocar_apply_warnings("${PROJECT_SOURCE_DIR}/cmake/config_check.c")

# --- Module source registration ----------------------------------------------
# Two source roots:
#   src/          platform code: common, rtos (port layer in src/rtos/port/),
#                 debug, app (src/app/main.c = SDK main(), Batch B)
#   app_program/  team code: app_main.c (usermain), subsystem1..5 (Buddy 1..5)
# Each directory in ROBOCAR_MODULE_DIRS (repo-relative) may provide a
# sources.cmake that appends ABSOLUTE paths (use "${CMAKE_CURRENT_LIST_DIR}/x.c"):
#   ROBOCAR_SOURCES         SDK-side files (may include pico/..., hardware/...)
#   ROBOCAR_KERNEL_SOURCES  kernel-side files (include <tk/tkernel.h>)
# A file is in exactly one list. No globbing of sources.
set(ROBOCAR_MODULE_DIRS
    src/common src/rtos src/debug src/app
    app_program
    app_program/subsystem1 app_program/subsystem2 app_program/subsystem3
    app_program/subsystem4 app_program/subsystem5)

# Fail on any sources.cmake that would be silently ignored: only
# <listed dir>/sources.cmake is registered. Checked: every top-level
# */sources.cmake and every sources.cmake anywhere under src/ and app_program/.
# ProjectTemplate/ (course template, tracked as reference only) is never
# scanned or built: its matches are dropped before the check.
set(ROBOCAR_NEVER_SCAN_REGEX "^ProjectTemplate/")
set(_registered "")
foreach(_dir IN LISTS ROBOCAR_MODULE_DIRS)
    list(APPEND _registered "${PROJECT_SOURCE_DIR}/${_dir}/sources.cmake")
endforeach()
file(GLOB _found_top "${PROJECT_SOURCE_DIR}/*/sources.cmake")
set(_found_tree "")
foreach(_root IN ITEMS src app_program)
    if(IS_DIRECTORY "${PROJECT_SOURCE_DIR}/${_root}")
        file(GLOB_RECURSE _found_root "${PROJECT_SOURCE_DIR}/${_root}/sources.cmake")
        list(APPEND _found_tree ${_found_root})
    endif()
endforeach()
foreach(_f IN LISTS _found_top _found_tree)
    file(RELATIVE_PATH _rel "${PROJECT_SOURCE_DIR}" "${_f}")
    if(_rel MATCHES "${ROBOCAR_NEVER_SCAN_REGEX}")
        continue()
    endif()
    list(FIND _registered "${_f}" _idx)
    if(_idx EQUAL -1)
        message(FATAL_ERROR "robocar: ${_rel} is not registered. sources.cmake files must be in a "
                            "directory listed in ROBOCAR_MODULE_DIRS (src/common, src/rtos, src/debug, "
                            "src/app, app_program, app_program/subsystem1..5).")
    endif()
endforeach()

set(ROBOCAR_SOURCES "")
set(ROBOCAR_KERNEL_SOURCES "")
foreach(_dir IN LISTS ROBOCAR_MODULE_DIRS)
    if(EXISTS "${PROJECT_SOURCE_DIR}/${_dir}/sources.cmake")
        include("${PROJECT_SOURCE_DIR}/${_dir}/sources.cmake")
    endif()
endforeach()

if(NOT EXISTS "${PROJECT_SOURCE_DIR}/src/app/main.c")
    # No SDK main() and no port layer until Batch B, so the firmware cannot
    # link. Compile every registered source instead, with -Werror, in ALL.
    message(STATUS "robocar: src/app/main.c not present (Batch B); firmware target skipped, app_program compile-checked")
    if(ROBOCAR_KERNEL_SOURCES)
        # Kernel side: kernel include paths only (no SDK include paths).
        add_library(app_program_check OBJECT ${ROBOCAR_KERNEL_SOURCES})
        target_link_libraries(app_program_check PRIVATE mtk3_headers robocar_options)
        set_target_properties(app_program_check PROPERTIES C_STANDARD 11 C_EXTENSIONS ON)
        robocar_apply_warnings(${ROBOCAR_KERNEL_SOURCES})
        add_dependencies(app_program_check check_rules)
    endif()
    if(ROBOCAR_SOURCES)
        # SDK side: hardware_* include paths without pulling the SDK runtime
        # (crt0, stdio) into a compile-only target.
        add_library(app_program_check_sdk OBJECT ${ROBOCAR_SOURCES})
        target_link_libraries(app_program_check_sdk PRIVATE
            robocar_options
            hardware_gpio
            hardware_adc
            hardware_pwm
            hardware_i2c
            hardware_irq
            hardware_sync
            hardware_pio
            hardware_exception
            hardware_clocks
            hardware_timer
            hardware_watchdog)
        set_target_properties(app_program_check_sdk PROPERTIES C_STANDARD 11 C_EXTENSIONS ON)
        robocar_apply_warnings(${ROBOCAR_SOURCES})
        add_dependencies(app_program_check_sdk check_rules)
    endif()
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
        hardware_pio
        hardware_uart
        hardware_exception
        hardware_clocks
        hardware_timer
        hardware_watchdog)
    if(TARGET robocar_kside)
        target_link_libraries(robocar PRIVATE robocar_kside)
    endif()

    pico_set_linker_script(robocar "${PROJECT_SOURCE_DIR}/cmake/memmap_mtk3.ld")

    # USB CDC only. stdio UART must never be enabled: it takes GP0/GP1 (pico_w
    # board default), which are the right wheel encoder.
    pico_enable_stdio_uart(robocar 0)
    pico_enable_stdio_usb(robocar 1)

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
