# app_program/subsystem1/sources.cmake: subsystem 1 source registration.
# ROBOCAR_KERNEL_SOURCES = kernel-side files (subsystem1.c, *_test.c, task_*.c; kernel headers only).
#                          tel_link.c, tel_outbox.c and tel_wire.c are pure C (no kernel or
#                          SDK headers); they sit on the kernel side with the task that uses them.
# ROBOCAR_SOURCES        = SDK-side drv_*.c files (SDK headers only, never subsystem1.h). Never both.
list(APPEND ROBOCAR_KERNEL_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/subsystem1.c"
    "${CMAKE_CURRENT_LIST_DIR}/subsystem1_test.c"
    "${CMAKE_CURRENT_LIST_DIR}/task_tel.c"
    "${CMAKE_CURRENT_LIST_DIR}/tel_link.c"
    "${CMAKE_CURRENT_LIST_DIR}/tel_outbox.c"
    "${CMAKE_CURRENT_LIST_DIR}/tel_wire.c")

# WiFi/MQTT driver: only with -DTEL_BACKEND_WIFI=ON (CYW43 + lwIP, poll mode).
# The top-level CMakeLists.txt links the SDK libraries and adds this folder to
# the include path for lwipopts.h and tel_cyw43_config.h.
if(TEL_BACKEND_WIFI)
    list(APPEND ROBOCAR_SOURCES "${CMAKE_CURRENT_LIST_DIR}/drv_wifi.c")
endif()
