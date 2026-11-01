# tests/CMakeLists.txt: host unit tests (plain C, host compiler, no board, no SDK).
# Configure standalone, never as part of the firmware:
#   cmake -S tests -B build/host -G Ninja
#   cmake --build build/host
#   ctest --test-dir build/host --output-on-failure
# tests/rules_fixtures/ holds rule-check fixtures: never compiled, never globbed.
cmake_minimum_required(VERSION 3.13)

# The top-level CMakeLists.txt adds this directory to the firmware build;
# host tests must not be built with the Arm cross compiler.
if(CMAKE_CROSSCOMPILING)
    return()
endif()

project(robocar_host_tests C)
enable_testing()

get_filename_component(ROBOCAR_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

# dbg_ ring (src/debug/dbg_ring.c), with a stand-in <tk/tkernel.h> (host/stubs).
add_executable(test_dbg_ring host/test_dbg_ring.c)
target_include_directories(test_dbg_ring PRIVATE host/stubs "${ROBOCAR_ROOT}")
target_compile_definitions(test_dbg_ring PRIVATE DBG_ENABLE=1)
target_compile_options(test_dbg_ring PRIVATE -Wall -Wextra -Werror)
set_target_properties(test_dbg_ring PROPERTIES C_STANDARD 11 C_EXTENSIONS ON)
add_test(NAME dbg_ring COMMAND test_dbg_ring)

# CRC-32 for the RAM fault record (src/common/crc32.h).
add_executable(test_crc32 host/test_crc32.c)
target_include_directories(test_crc32 PRIVATE "${ROBOCAR_ROOT}")
target_compile_options(test_crc32 PRIVATE -Wall -Wextra -Werror)
set_target_properties(test_crc32 PROPERTIES C_STANDARD 11 C_EXTENSIONS ON)
add_test(NAME crc32 COMMAND test_crc32)

# Periodic wait (src/common/period.c), against a simulated kernel tick.
add_executable(test_period host/test_period.c "${ROBOCAR_ROOT}/src/common/period.c")
target_include_directories(test_period PRIVATE host/stubs "${ROBOCAR_ROOT}")
target_compile_options(test_period PRIVATE -Wall -Wextra -Werror)
set_target_properties(test_period PROPERTIES C_STANDARD 11 C_EXTENSIONS ON)
add_test(NAME period COMMAND test_period)

# Packed data representation (src/common/data_repr.h, stage B7): every
# format at its limits, sign extension, edge ring, MQTT frames and rejects.
add_executable(test_data_repr host/test_data_repr.c)
target_include_directories(test_data_repr PRIVATE "${ROBOCAR_ROOT}")
target_compile_options(test_data_repr PRIVATE -Wall -Wextra -Werror)
set_target_properties(test_data_repr PROPERTIES C_STANDARD 11 C_EXTENSIONS ON)
add_test(NAME data_repr COMMAND test_data_repr)

# Code 39 barcode decoder core (app_program/subsystem3/bc_decode.c, pure C):
# Subsystem 3 test S3-U01. Fixtures model the barcode, the car's motion and
# Motion Control's published speed (needs libm for the physics).
add_executable(test_bc_decode host/test_bc_decode.c
    "${ROBOCAR_ROOT}/app_program/subsystem3/bc_decode.c")
target_include_directories(test_bc_decode PRIVATE "${ROBOCAR_ROOT}")
target_compile_options(test_bc_decode PRIVATE -Wall -Wextra -Werror)
target_link_libraries(test_bc_decode PRIVATE m)
set_target_properties(test_bc_decode PROPERTIES C_STANDARD 11 C_EXTENSIONS ON)
add_test(NAME bc_decode COMMAND test_bc_decode)

# Barcode capture framing (app_program/subsystem3/bc_frame.c + bc_decode.c,
# pure C): Subsystem 3 test S3-U02. Same barcode/physics/Motion generator as
# S3-U01 (host/bc_fixture.h), plus the Barcode task's poll rule.
add_executable(test_bc_frame host/test_bc_frame.c
    "${ROBOCAR_ROOT}/app_program/subsystem3/bc_frame.c"
    "${ROBOCAR_ROOT}/app_program/subsystem3/bc_decode.c")
target_include_directories(test_bc_frame PRIVATE "${ROBOCAR_ROOT}")
target_compile_options(test_bc_frame PRIVATE -Wall -Wextra -Werror)
target_link_libraries(test_bc_frame PRIVATE m)
set_target_properties(test_bc_frame PROPERTIES C_STANDARD 11 C_EXTENSIONS ON)
add_test(NAME bc_frame COMMAND test_bc_frame)
