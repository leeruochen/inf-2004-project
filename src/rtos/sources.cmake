# src/rtos/sources.cmake: kernel port layer (src/rtos/port/; a sources.cmake
# there would not be registered, CMakeLists.txt ROBOCAR_MODULE_DIRS).
# ROBOCAR_KERNEL_SOURCES = kernel-side files (kernel headers only).
# ROBOCAR_SOURCES        = SDK-side files (SDK headers only). Never both.
list(APPEND ROBOCAR_KERNEL_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/port/port_kernel_internal.c")
list(APPEND ROBOCAR_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/port/port_boot.c"
    "${CMAKE_CURRENT_LIST_DIR}/port/port_fault.c"
    "${CMAKE_CURRENT_LIST_DIR}/port/port_irq.c")
