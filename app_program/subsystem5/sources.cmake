# app_program/subsystem5/sources.cmake: subsystem 5 source registration.
# ROBOCAR_KERNEL_SOURCES = kernel-side files (subsystem5.c, *_test.c, task_*.c; kernel headers only).
# ROBOCAR_SOURCES        = SDK-side drv_*.c files (SDK headers only, never subsystem5.h). Never both.
list(APPEND ROBOCAR_KERNEL_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/subsystem5.c"
    "${CMAKE_CURRENT_LIST_DIR}/subsystem5_test.c"
    "${CMAKE_CURRENT_LIST_DIR}/task_obs_scan.c"
    "${CMAKE_CURRENT_LIST_DIR}/task_obs_avoid.c")
# SDK-side drivers:
list(APPEND ROBOCAR_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/drv_servo.c"
    "${CMAKE_CURRENT_LIST_DIR}/drv_ultrasonic.c")
