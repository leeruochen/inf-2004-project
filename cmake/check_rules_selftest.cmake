# cmake/check_rules_selftest.cmake
#
# Proves check_rules.cmake actually fails. Runs it against the fixtures in
# tests/rules_fixtures/ (never compiled) and checks every rule fired, and that
# the allowed cases did not. Run with:
#   cmake -P cmake/check_rules_selftest.cmake

cmake_minimum_required(VERSION 3.13)

execute_process(
    COMMAND "${CMAKE_COMMAND}"
            "-DRULES_ROOT=${CMAKE_CURRENT_LIST_DIR}/../tests/rules_fixtures"
            -P "${CMAKE_CURRENT_LIST_DIR}/check_rules.cmake"
    RESULT_VARIABLE rc
    OUTPUT_VARIABLE out
    ERROR_VARIABLE err)

set(all "${out}${err}")

if(rc EQUAL 0)
    message(FATAL_ERROR "check_rules_selftest: check_rules passed on the fixtures; it must fail\n${all}")
endif()

set(expected
    "app_program/subsystem2/r1_mixed.c:" "[R1]"
    "app_program/subsystem2/r2_blocking.c:2:" "app_program/subsystem2/r2_blocking.c:3:" "[R2]"
    "app_program/subsystem3/r3_stdio.c:2:" "app_program/subsystem3/r3_stdio.c:4:" "[R3a]" "[R3b]"
    "app_program/subsystem2/r4_libbsp.c:" "[R4]"
    "app_program/subsystem4/r5_core1.c:" "[R5]"
    "app_program/subsystem4/r6_task_exit.c:2: [R6]" "app_program/subsystem4/r6_task_exit.c:3: [R6]"
    "app_program/subsystem2/r2_pio_blocking.c:2:" "app_program/subsystem2/r2_pio_blocking.c:3:"
    "app_program/subsystem2/r1_drv_subsystem_hdr.c:3: [R1]"
    "app_program/subsystem2/r1_drv_ipc_hdr.c:3: [R1]"
    "tools/r2_unlisted_dir.c:2:"
    "r3_root_file.c:2:"
    "app_program/subsystem1/r3_app_program.c:2: [R3b]")
foreach(e IN LISTS expected)
    string(FIND "${all}" "${e}" pos)
    if(pos EQUAL -1)
        message(FATAL_ERROR "check_rules_selftest: expected '${e}' in output\n${all}")
    endif()
endforeach()

set(unexpected "src/app/main.c" "snprintf" "app_program/subsystem3/r3_stdio.c:3:" "docs/excluded.c"
    "app_program/subsystem4/r6_task_exit.c:4:"
    "ProjectTemplate/")
foreach(u IN LISTS unexpected)
    string(FIND "${all}" "${u}" pos)
    if(NOT pos EQUAL -1)
        message(FATAL_ERROR "check_rules_selftest: '${u}' must not be reported\n${all}")
    endif()
endforeach()

message(STATUS "check_rules_selftest: PASS")
