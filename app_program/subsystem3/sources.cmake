# app_program/subsystem3/sources.cmake: subsystem 3 source registration.
# ROBOCAR_KERNEL_SOURCES = kernel-side files (subsystem3.c, *_test.c, task_*.c; kernel headers only).
# ROBOCAR_SOURCES        = SDK-side drv_*.c files (SDK headers only, never subsystem3.h). Never both.
# bc_decode.c and bc_frame.c are pure C (no kernel or SDK header; host-tested
# by tests/host/test_bc_decode.c and test_bc_frame.c); they sit on the kernel
# side with their caller, the Barcode Decode task.
# lf_core.c and lf_link.c are pure C (host-tested by tests/host/
# test_lf_core.c, S3-U10, and test_lf_link.c, S3-U11); they sit on the
# kernel side with their caller, the Line-Following task (task_lf.c).
# lf_run_test.c is the on-target S3-U12 (APP_TEST builds only).
# lf_step_test.c is the on-target S3-U16 diagnostic (APP_TEST, -DS3_STEP_DIAG=1).
list(APPEND ROBOCAR_KERNEL_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/subsystem3.c"
    "${CMAKE_CURRENT_LIST_DIR}/subsystem3_test.c"
    "${CMAKE_CURRENT_LIST_DIR}/lf_hw_test.c"
    "${CMAKE_CURRENT_LIST_DIR}/bc_decode.c"
    "${CMAKE_CURRENT_LIST_DIR}/bc_frame.c"
    "${CMAKE_CURRENT_LIST_DIR}/lf_core.c"
    "${CMAKE_CURRENT_LIST_DIR}/lf_link.c"
    "${CMAKE_CURRENT_LIST_DIR}/task_lf.c"
    "${CMAKE_CURRENT_LIST_DIR}/lf_run_test.c"
    "${CMAKE_CURRENT_LIST_DIR}/lf_step_test.c")
# SDK-side drivers:
# lf_sanity.c is pure C (host-tested by tests/host/test_lf_sanity.c); it sits
# on the SDK side with its caller, drv_ir.c.
list(APPEND ROBOCAR_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/drv_ir.c"
    "${CMAKE_CURRENT_LIST_DIR}/lf_sanity.c")
# -DS3_STEP_DIAG=ON (APP_TEST builds): subsystem3_test() runs the S3-U16
# steering step diagnostic on the open floor in place of the track test.
# -DS3_TURN_DIAG=ON: S3-U18, the turning radius diagnostic, likewise.
# -DS3_CURVE_TEST=ON: the track test runs the S3-U17 curve drives in place of
# the S3-U14 U-turn.  At most one of the three; all OFF = the U-turn test.
option(S3_STEP_DIAG "Subsystem 3 APP_TEST: run the S3-U16 steering step diagnostic, not the track test" OFF)
option(S3_TURN_DIAG "Subsystem 3 APP_TEST: run the S3-U18 turning radius diagnostic, not the track test" OFF)
option(S3_CURVE_TEST "Subsystem 3 APP_TEST: the track test runs the S3-U17 curve drives" OFF)
set(_s3_modes 0)
foreach(_m S3_STEP_DIAG S3_TURN_DIAG S3_CURVE_TEST)
    if(${_m})
        math(EXPR _s3_modes "${_s3_modes} + 1")
    endif()
endforeach()
if(_s3_modes GREATER 1)
    message(FATAL_ERROR "Subsystem 3: S3_STEP_DIAG, S3_TURN_DIAG and S3_CURVE_TEST are exclusive")
endif()
if(S3_STEP_DIAG)
    set_property(SOURCE "${CMAKE_CURRENT_LIST_DIR}/subsystem3_test.c"
        APPEND PROPERTY COMPILE_DEFINITIONS "S3_STEP_DIAG=1")
endif()
if(S3_TURN_DIAG)
    set_property(SOURCE "${CMAKE_CURRENT_LIST_DIR}/subsystem3_test.c"
        APPEND PROPERTY COMPILE_DEFINITIONS "S3_TURN_DIAG=1")
endif()
if(S3_CURVE_TEST)
    set_property(SOURCE "${CMAKE_CURRENT_LIST_DIR}/lf_run_test.c"
        APPEND PROPERTY COMPILE_DEFINITIONS "S3_CURVE_TEST=1")
endif()
