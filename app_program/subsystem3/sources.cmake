# app_program/subsystem3/sources.cmake: subsystem 3 source registration.
# ROBOCAR_KERNEL_SOURCES = kernel-side files (subsystem3.c, *_test.c, task_*.c; kernel headers only).
# ROBOCAR_SOURCES        = SDK-side drv_*.c files (SDK headers only, never subsystem3.h). Never both.
# bc_decode.c and bc_frame.c are pure C (no kernel or SDK header; host-tested
# by tests/host/test_bc_decode.c and test_bc_frame.c); they sit on the kernel
# side with their caller, the Barcode Decode task.
# lf_core.c and lf_link.c are pure C (host-tested by tests/host/
# test_lf_core.c, S3-U10, and test_lf_link.c, S3-U11); they sit on the
# kernel side with their caller, the Line-Following task (task_lf.c).
# lf_straddle.c (ARCHITECTURE.md D14) is pure C, host-tested by tests/host/
# test_lf_straddle.c: the straddle core, used in place of lf_core.c's steering
# with -DS3_LF_STRADDLE=ON (always compiled, so the rules check it).
# lf_run_test.c is the on-target S3-U12 (APP_TEST builds only).
# lf_step_test.c is the on-target S3-U16 diagnostic (APP_TEST, -DS3_STEP_DIAG=1).
# bc_edge.c is pure C too (tests/host/test_bc_edge.c); it sits on the SDK side with
# its caller, the GP27 handler in drv_bc.c.
list(APPEND ROBOCAR_KERNEL_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/subsystem3.c"
    "${CMAKE_CURRENT_LIST_DIR}/subsystem3_test.c"
    "${CMAKE_CURRENT_LIST_DIR}/lf_hw_test.c"
    "${CMAKE_CURRENT_LIST_DIR}/bc_decode.c"
    "${CMAKE_CURRENT_LIST_DIR}/bc_frame.c"
    "${CMAKE_CURRENT_LIST_DIR}/lf_core.c"
    "${CMAKE_CURRENT_LIST_DIR}/lf_link.c"
    "${CMAKE_CURRENT_LIST_DIR}/lf_straddle.c"
    "${CMAKE_CURRENT_LIST_DIR}/task_lf.c"
    "${CMAKE_CURRENT_LIST_DIR}/lf_run_test.c"
    "${CMAKE_CURRENT_LIST_DIR}/lf_step_test.c"
    "${CMAKE_CURRENT_LIST_DIR}/task_bc.c")
# SDK-side drivers:
# lf_sanity.c is pure C (host-tested by tests/host/test_lf_sanity.c); it sits
# on the SDK side with its caller, drv_ir.c.
list(APPEND ROBOCAR_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/drv_ir.c"
    "${CMAKE_CURRENT_LIST_DIR}/lf_sanity.c"
    "${CMAKE_CURRENT_LIST_DIR}/drv_bc.c"
    "${CMAKE_CURRENT_LIST_DIR}/bc_edge.c")
# -DS3_STEP_DIAG=ON (APP_TEST builds): subsystem3_test() runs the S3-U16
# steering step diagnostic on the open floor in place of the track test.
# -DS3_TURN_DIAG=ON: S3-U18, the turning radius diagnostic, likewise.
# -DS3_CURVE_TEST=ON: the track test runs the S3-U17 curve drives in place of
# the S3-U14 U-turn.  At most one of the three; all OFF = the U-turn test.
option(S3_STEP_DIAG "Subsystem 3 APP_TEST: run the S3-U16 steering step diagnostic, not the track test" OFF)
option(S3_TURN_DIAG "Subsystem 3 APP_TEST: run the S3-U18 turning radius diagnostic, not the track test" OFF)
option(S3_CURVE_TEST "Subsystem 3 APP_TEST: the track test runs the S3-U17 curve drives" OFF)
option(S3_BC_TEST "Subsystem 3 APP_TEST: run the barcode tests S3-U03..U06 only, not the line-following track test" OFF)
option(S3_BC_QUICK "Subsystem 3 APP_TEST: quick barcode drive test, one card driven over 6 times" OFF)
# S3-U14 junction turn: S3_TURN = LEFT | RIGHT | UTURN (default UTURN); S3_QUICK_CAL = no pose A/B, a fixed TEST calibration.
set(S3_TURN "UTURN" CACHE STRING "Subsystem 3 APP_TEST: the junction turn the S3-U14 test makes: LEFT, RIGHT or UTURN")
option(S3_QUICK_CAL "Subsystem 3 APP_TEST: skip pose A and B, use a fixed test calibration (junction turn test)" OFF)
if(NOT S3_TURN MATCHES "^(LEFT|RIGHT|UTURN)$")
    message(FATAL_ERROR "S3_TURN='${S3_TURN}': LEFT, RIGHT or UTURN")
endif()
if(S3_TURN STREQUAL "LEFT")
    set_property(SOURCE "${CMAKE_CURRENT_LIST_DIR}/lf_run_test.c" APPEND PROPERTY COMPILE_DEFINITIONS "S3_TURN_LEFT=1")
elseif(S3_TURN STREQUAL "RIGHT")
    set_property(SOURCE "${CMAKE_CURRENT_LIST_DIR}/lf_run_test.c" APPEND PROPERTY COMPILE_DEFINITIONS "S3_TURN_RIGHT=1")
endif()
if(S3_QUICK_CAL)
    set_property(SOURCE "${CMAKE_CURRENT_LIST_DIR}/lf_run_test.c" APPEND PROPERTY COMPILE_DEFINITIONS "S3_QUICK_CAL=1")
endif()
set(S3_DRIVE_MM "" CACHE STRING "Subsystem 3 APP_TEST: controlled moving drives of up to this many mm (team moving tests, S3-U19); empty = off")
set(_s3_modes 0)
foreach(_m S3_STEP_DIAG S3_TURN_DIAG S3_CURVE_TEST S3_BC_TEST S3_BC_QUICK)
    if(${_m})
        math(EXPR _s3_modes "${_s3_modes} + 1")
    endif()
endforeach()
if(NOT S3_DRIVE_MM STREQUAL "")
    if(NOT S3_DRIVE_MM MATCHES "^[0-9]+$" OR S3_DRIVE_MM LESS 300 OR S3_DRIVE_MM GREATER 20000)
        message(FATAL_ERROR "S3_DRIVE_MM='${S3_DRIVE_MM}': whole millimetres, 300..20000")
    endif()
    math(EXPR _s3_modes "${_s3_modes} + 1")
    set_property(SOURCE "${CMAKE_CURRENT_LIST_DIR}/lf_run_test.c"
        APPEND PROPERTY COMPILE_DEFINITIONS "S3_DRIVE_MM=${S3_DRIVE_MM}")
endif()
if(_s3_modes GREATER 1)
    message(FATAL_ERROR "Subsystem 3: S3_STEP_DIAG, S3_TURN_DIAG, S3_CURVE_TEST, S3_BC_TEST, S3_BC_QUICK and S3_DRIVE_MM are exclusive")
endif()
if(S3_STEP_DIAG)
    set_property(SOURCE "${CMAKE_CURRENT_LIST_DIR}/subsystem3_test.c"
        APPEND PROPERTY COMPILE_DEFINITIONS "S3_STEP_DIAG=1")
endif()
if(S3_TURN_DIAG)
    set_property(SOURCE "${CMAKE_CURRENT_LIST_DIR}/subsystem3_test.c"
        APPEND PROPERTY COMPILE_DEFINITIONS "S3_TURN_DIAG=1")
endif()
if(S3_BC_TEST)
    set_property(SOURCE "${CMAKE_CURRENT_LIST_DIR}/subsystem3_test.c"
        APPEND PROPERTY COMPILE_DEFINITIONS "S3_BC_TEST=1")
endif()
if(S3_BC_QUICK)
    set_property(SOURCE "${CMAKE_CURRENT_LIST_DIR}/subsystem3_test.c"
        APPEND PROPERTY COMPILE_DEFINITIONS "S3_BC_QUICK=1")
endif()
if(S3_CURVE_TEST)
    set_property(SOURCE "${CMAKE_CURRENT_LIST_DIR}/lf_run_test.c"
        APPEND PROPERTY COMPILE_DEFINITIONS "S3_CURVE_TEST=1")
endif()
# -DS3_LF_STRADDLE=ON (ARCHITECTURE.md D14): the
# Line-Following task runs the straddle core (lf_straddle.c), the sensors
# re-mounted one each side of the line; OFF = the edge-following core.
option(S3_LF_STRADDLE "Subsystem 3: straddle line following (sensors re-mounted, D14)" OFF)
if(S3_LF_STRADDLE)
    set_property(SOURCE "${CMAKE_CURRENT_LIST_DIR}/task_lf.c" "${CMAKE_CURRENT_LIST_DIR}/lf_run_test.c"
        APPEND PROPERTY COMPILE_DEFINITIONS "S3_LF_STRADDLE=1")
endif()
