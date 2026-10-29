# app_program/subsystem4/sources.cmake: subsystem 4 source registration.
# ROBOCAR_KERNEL_SOURCES = kernel-side files (subsystem4.c, *_test.c, task_*.c; kernel headers only).
# ROBOCAR_SOURCES        = SDK-side drv_*.c files (SDK headers only, never subsystem4.h). Never both.
list(APPEND ROBOCAR_KERNEL_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/subsystem4.c"
    "${CMAKE_CURRENT_LIST_DIR}/subsystem4_test.c")
# SDK-side drivers go here, for example:
#   list(APPEND ROBOCAR_SOURCES "${CMAKE_CURRENT_LIST_DIR}/drv_example.c")
