# app_program/sources.cmake: shared team files (usermain).
# ROBOCAR_KERNEL_SOURCES = kernel-side files (kernel headers only).
# ROBOCAR_SOURCES        = SDK-side drv_*.c files (SDK headers only). Never both.
list(APPEND ROBOCAR_KERNEL_SOURCES "${CMAKE_CURRENT_LIST_DIR}/app_main.c")
