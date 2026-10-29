# cmake/mtk3.cmake
#
# micro T-Kernel 3.0 (TRON Forum mtk3_bsp, branch pico_rp2040) as a STATIC
# library `mtk3`, plus an INTERFACE target `mtk3_headers` for kernel-side
# project sources.
#
# Integration approach (a): the Pico SDK owns crt0, boot2, clocks, the linker
# script and the vector table. The kernel is a library behind the port layer
# (src/rtos/port/, Batch B). Vendored files are never edited; the kernel config is
# our copy in config/mtk3/, which is FIRST on the include path.
#
# mtk3 never sees an SDK header and never links pico_stdlib. It carries no
# PUBLIC/INTERFACE usage requirements, so nothing leaks into SDK-side code.
#
# Unresolved symbols, supplied by the port in Batch B (expected for a static lib):
#   knl_lowmem_top, knl_lowmem_limit, knl_shutdown_hw, knl_restart_hw,
#   tm_com_init, tm_snd_dat, tm_rcv_dat, usermain.
#   __aeabi_* integer division comes from the SDK (pico_divider).
#
# sysinit.c is compiled with -Dmain=mtk3_kernel_main so the kernel's boot
# sequence does not clash with the SDK main().

set(MTK3_ROOT "${PROJECT_SOURCE_DIR}/external/mtk3_bsp")
set(MTK3_PINNED_COMMIT 15ed232c08f2d89e54e514db79de150e224307e3)

option(MTK3_ALLOW_UNPINNED "Allow external/mtk3_bsp off the pinned commit or dirty (experiments only)" OFF)

# --- Submodule guards (configure time) -------------------------------------
if(NOT EXISTS "${MTK3_ROOT}/kernel/tkernel/task.c")
    message(FATAL_ERROR
        "mtk3: external/mtk3_bsp is missing. Run:\n"
        "  git submodule update --init --recursive\n"
        "See docs/SUBMODULE.md.")
endif()

find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${MTK3_ROOT}/.git")
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" -C "${MTK3_ROOT}" rev-parse HEAD
        OUTPUT_VARIABLE _mtk3_head
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE _mtk3_rc)
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" -C "${MTK3_ROOT}" status --porcelain
        OUTPUT_VARIABLE _mtk3_dirty
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    set(_mtk3_problem "")
    if(NOT _mtk3_rc EQUAL 0 OR NOT _mtk3_head STREQUAL MTK3_PINNED_COMMIT)
        set(_mtk3_problem "external/mtk3_bsp HEAD is '${_mtk3_head}', expected ${MTK3_PINNED_COMMIT}.")
    elseif(NOT _mtk3_dirty STREQUAL "")
        set(_mtk3_problem "external/mtk3_bsp has local changes. Vendored files must not be edited.")
    endif()
    if(_mtk3_problem)
        if(MTK3_ALLOW_UNPINNED)
            message(WARNING "mtk3: ${_mtk3_problem} (MTK3_ALLOW_UNPINNED=ON)")
        else()
            message(FATAL_ERROR "mtk3: ${_mtk3_problem} See docs/SUBMODULE.md.")
        endif()
    else()
        message(STATUS "mtk3: external/mtk3_bsp pinned at ${MTK3_PINNED_COMMIT}, clean")
    endif()
else()
    message(WARNING "mtk3: git not found or submodule metadata missing; pin not verified")
endif()

# --- Source list (report section 11, 43 files, explicit) --------------------
#
# EXCLUDED, and why:
#   kernel/sysdepend/cpu/core/armv6m/reset_hdl.c  SDK crt0 owns reset
#   kernel/sysdepend/cpu/rp2040/vector_tbl.c      SDK owns the vector table
#   kernel/sysdepend/cpu/rp2040/boot_2nd.c        SDK owns boot2
#   kernel/sysdepend/pico_rp2040/cpu_clock.c      SDK owns clocks
#   kernel/sysdepend/pico_rp2040/hw_setting.c     SDK owns pin mux; drives GP25 = CYW43 CS on Pico W
#   lib/libtm/sysdepend/pico_rp2040/tm_com.c      port provides tm_com_init/tm_snd_dat/tm_rcv_dat
#   lib/libbsp/sysdepend/cpu/rp2040/libbsp_pwm.c  never link: clashes with hardware/pwm.h # RULE-ALLOW: exclusion-note
#   device/**, app_program/**                     the BSP's own sample drivers and sample app in
#                                                 external/mtk3_bsp (not our app_program/), not used
#   kernel/usermain/usermain.c                    we provide usermain
set(MTK3_SOURCES_REL
    kernel/tkernel/cpuctl.c
    kernel/tkernel/device.c
    kernel/tkernel/deviceio.c
    kernel/tkernel/eventflag.c
    kernel/tkernel/int.c
    kernel/tkernel/klock.c
    kernel/tkernel/mailbox.c
    kernel/tkernel/memory.c
    kernel/tkernel/mempfix.c
    kernel/tkernel/mempool.c
    kernel/tkernel/messagebuf.c
    kernel/tkernel/misc_calls.c
    kernel/tkernel/mutex.c
    kernel/tkernel/objname.c
    kernel/tkernel/power.c
    kernel/tkernel/rendezvous.c
    kernel/tkernel/semaphore.c
    kernel/tkernel/task.c
    kernel/tkernel/task_manage.c
    kernel/tkernel/task_sync.c
    kernel/tkernel/timer.c
    kernel/tkernel/time_calls.c
    kernel/tkernel/tkinit.c
    kernel/tkernel/wait.c
    kernel/tstdlib/bitop.c
    kernel/tstdlib/string.c
    kernel/sysinit/sysinit.c
    kernel/inittask/inittask.c
    kernel/sysdepend/cpu/core/armv6m/cpu_cntl.c
    kernel/sysdepend/cpu/core/armv6m/exc_hdr.c
    kernel/sysdepend/cpu/core/armv6m/interrupt.c
    kernel/sysdepend/cpu/core/armv6m/dispatch.S
    kernel/sysdepend/pico_rp2040/devinit.c
    kernel/sysdepend/pico_rp2040/power_save.c
    lib/libtk/fastlock.c
    lib/libtk/fastmlock.c
    lib/libtk/kmalloc.c
    lib/libtk/sysdepend/cpu/core/armv6m/int_armv6m.c
    lib/libtk/sysdepend/cpu/core/armv6m/wusec_armv6m.c
    lib/libtk/sysdepend/cpu/rp2040/int_rp2040.c
    lib/libtk/sysdepend/cpu/rp2040/ptimer_rp2040.c
    lib/libtm/libtm.c
    lib/libtm/libtm_printf.c
)

# Self-guards: every file exists, and nothing from the exclusion list slipped in.
set(_mtk3_forbidden ${MTK3_SOURCES_REL})
list(FILTER _mtk3_forbidden INCLUDE REGEX
    "reset_hdl|vector_tbl|boot_2nd|cpu_clock|hw_setting|tm_com|libbsp|/device/|^device/|app_program|usermain")
if(_mtk3_forbidden)
    message(FATAL_ERROR "mtk3: excluded file(s) in MTK3_SOURCES: ${_mtk3_forbidden}")
endif()

set(MTK3_SOURCES "")
foreach(_src IN LISTS MTK3_SOURCES_REL)
    if(NOT EXISTS "${MTK3_ROOT}/${_src}")
        message(FATAL_ERROR "mtk3: missing kernel source ${MTK3_ROOT}/${_src}")
    endif()
    list(APPEND MTK3_SOURCES "${MTK3_ROOT}/${_src}")
endforeach()

# --- mtk3 static library -----------------------------------------------------
add_library(mtk3 STATIC ${MTK3_SOURCES})

# Order matters: our config/mtk3 first so <config.h> resolves to the project copy.
target_include_directories(mtk3 PRIVATE
    "${PROJECT_SOURCE_DIR}/config/mtk3"
    "${MTK3_ROOT}/include"
    "${MTK3_ROOT}/kernel/knlinc")

target_compile_definitions(mtk3 PRIVATE _PICO_RP2040_)

# No -Werror on vendored code. The -Wno-* flags silence known kernel warnings
# (report section 11) on mtk3 only.
target_compile_options(mtk3 PRIVATE
    -mcpu=cortex-m0plus -mthumb
    -ffunction-sections -fdata-sections
    $<$<COMPILE_LANGUAGE:C>:-ffreestanding -fsigned-char -std=gnu11 -Wall -Wextra
                            -Wno-unused-parameter -Wno-sign-compare -Wno-cast-function-type>)

set_target_properties(mtk3 PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON C_EXTENSIONS ON)

set_source_files_properties("${MTK3_ROOT}/kernel/sysinit/sysinit.c"
    PROPERTIES COMPILE_DEFINITIONS "main=mtk3_kernel_main")

# --- mtk3_headers: kernel API include paths for kernel-side project code -----
# Link ONLY from the kernel-side OBJECT library (robocar_kside), never from
# SDK-side code. A source file includes <tk/tkernel.h> or SDK headers, never both.
add_library(mtk3_headers INTERFACE)
target_include_directories(mtk3_headers INTERFACE
    "${PROJECT_SOURCE_DIR}/config/mtk3"
    "${MTK3_ROOT}/include")
target_compile_definitions(mtk3_headers INTERFACE _PICO_RP2040_)
