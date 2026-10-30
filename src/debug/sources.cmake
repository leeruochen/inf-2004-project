# src/debug/sources.cmake: debug console (dbg_ ring; the drain task arrives in stage B4).
# ROBOCAR_KERNEL_SOURCES = kernel-side files (kernel headers only).
# Built only with DBG_ENABLE=ON; with OFF, src/debug/dbg.h turns every dbg_ call into nothing.
if(DBG_ENABLE)
    list(APPEND ROBOCAR_KERNEL_SOURCES "${CMAKE_CURRENT_LIST_DIR}/dbg_ring.c")
endif()
