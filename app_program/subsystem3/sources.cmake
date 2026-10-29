# app_program/subsystem3/sources.cmake: subsystem 3 source registration.
# ROBOCAR_KERNEL_SOURCES = kernel-side files (subsystem3.c, *_test.c, task_*.c; kernel headers only).
# ROBOCAR_SOURCES        = SDK-side drv_*.c files (SDK headers only, never subsystem3.h). Never both.
list(APPEND ROBOCAR_KERNEL_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/subsystem3.c"
    "${CMAKE_CURRENT_LIST_DIR}/subsystem3_test.c")
# SDK-side drivers:
list(APPEND ROBOCAR_SOURCES "${CMAKE_CURRENT_LIST_DIR}/drv_ir.c")
