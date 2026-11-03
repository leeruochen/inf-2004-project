# app_program/subsystem3/sources.cmake: subsystem 3 source registration.
# ROBOCAR_KERNEL_SOURCES = kernel-side files (subsystem3.c, *_test.c, task_*.c; kernel headers only).
# ROBOCAR_SOURCES        = SDK-side drv_*.c files (SDK headers only, never subsystem3.h). Never both.
# bc_decode.c and bc_frame.c are pure C (no kernel or SDK header; host-tested
# by tests/host/test_bc_decode.c and test_bc_frame.c); they sit on the kernel
# side with their caller, the Barcode Decode task.
# lf_core.c is pure C (host-tested by tests/host/test_lf_core.c, S3-U10); it
# sits on the kernel side with its future caller, the Line-Following task
# (LF-3).  Nothing calls it yet; it is built so the gates compile and
# rule-check it.
list(APPEND ROBOCAR_KERNEL_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/subsystem3.c"
    "${CMAKE_CURRENT_LIST_DIR}/subsystem3_test.c"
    "${CMAKE_CURRENT_LIST_DIR}/lf_hw_test.c"
    "${CMAKE_CURRENT_LIST_DIR}/bc_decode.c"
    "${CMAKE_CURRENT_LIST_DIR}/bc_frame.c"
    "${CMAKE_CURRENT_LIST_DIR}/lf_core.c")
# SDK-side drivers:
# lf_sanity.c is pure C (host-tested by tests/host/test_lf_sanity.c); it sits
# on the SDK side with its caller, drv_ir.c.
list(APPEND ROBOCAR_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/drv_ir.c"
    "${CMAKE_CURRENT_LIST_DIR}/lf_sanity.c")
