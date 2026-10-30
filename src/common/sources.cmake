# src/common/sources.cmake: IPC objects (ipc_init) and task stack painting.
# ROBOCAR_KERNEL_SOURCES = kernel-side files (kernel headers only).
list(APPEND ROBOCAR_KERNEL_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/ipc.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack_mon.c")
