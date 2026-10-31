# app_program/subsystem2/sources.cmake: subsystem 2 source registration.
# ROBOCAR_KERNEL_SOURCES = kernel-side files (subsystem2.c, *_test.c, task_*.c; kernel headers only).
# ROBOCAR_SOURCES        = SDK-side drv_*.c files (SDK headers only, never subsystem2.h). Never both.
list(APPEND ROBOCAR_KERNEL_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/subsystem2.c"
    "${CMAKE_CURRENT_LIST_DIR}/subsystem2_test.c"
    "${CMAKE_CURRENT_LIST_DIR}/task_motion.c"
    "${CMAKE_CURRENT_LIST_DIR}/task_vc.c")
# SDK-side drivers:
list(APPEND ROBOCAR_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/drv_motor.c"
    "${CMAKE_CURRENT_LIST_DIR}/drv_encoder.c")
