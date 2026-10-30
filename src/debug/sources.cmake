# src/debug/sources.cmake: debug console (dbg_ ring and the drain task).
# ROBOCAR_KERNEL_SOURCES = kernel-side files (kernel headers only; dbg_drain.c also <stdio.h>).
# Built only with DBG_ENABLE=ON; with OFF, src/debug/dbg.h turns every dbg_ call into nothing.
if(DBG_ENABLE)
    list(APPEND ROBOCAR_KERNEL_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/dbg_ring.c"
        "${CMAKE_CURRENT_LIST_DIR}/dbg_drain.c")
endif()
