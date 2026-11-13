# app_program/subsystem4/sources.cmake: subsystem 4 source registration.
# ROBOCAR_KERNEL_SOURCES = kernel-side files (subsystem4.c, *_test.c, task_*.c; kernel headers only).
# ROBOCAR_SOURCES        = SDK-side drv_*.c files (SDK headers only, never subsystem4.h). Never both.
list(APPEND ROBOCAR_KERNEL_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/subsystem4.c"
    "${CMAKE_CURRENT_LIST_DIR}/imu_lift.c"
    "${CMAKE_CURRENT_LIST_DIR}/subsystem4_test.c"
    "${CMAKE_CURRENT_LIST_DIR}/task_imu.c"
    "${CMAKE_CURRENT_LIST_DIR}/task_safety.c")
list(APPEND ROBOCAR_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/drv_imu.c")
# imu_lift.c is pure C (host-tested by tests/host/test_imu_lift.c, S4-U21); it
# sits on the kernel side with its caller, the IMU Processing task.
# Operator-guided tests S4-U14..U16 and S4-U18..U20 (subsystem4_test.c), off by default:
#   cmake ... -DAPP_TEST=ON -DS4_MANUAL_TESTS=ON
option(S4_MANUAL_TESTS "Subsystem 4 operator-guided on-target tests (APP_TEST builds)" OFF)
if(S4_MANUAL_TESTS)
    set_property(SOURCE "${CMAKE_CURRENT_LIST_DIR}/subsystem4_test.c"
                 APPEND PROPERTY COMPILE_DEFINITIONS S4_MANUAL_TESTS=1)
endif()
