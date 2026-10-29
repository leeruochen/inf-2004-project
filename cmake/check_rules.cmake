# cmake/check_rules.cmake
#
# Project rule checks (CI-style grep). Run with:
#   cmake -P cmake/check_rules.cmake
# or build the `check_rules` target (runs automatically before robocar).
# Optional: -DRULES_ROOT=<dir> to scan another tree (used by the self-test).
#
# Rules (FAIL with <file>:<line>):
#   R1  A file includes both a kernel header (<tk/..>, <tm/..>, <bsp/..>,
#       <sys/{inittask,knldef,machine,profile,queue,sysdef}.h>, <sys/sysdepend/..>,
#       config/mtk3/.., or our kernel-side project headers "app.h",
#       "test_report.h", "subsystemN.h" with any relative path) and an SDK header
#       (pico/.., pico.h, hardware/.., boards/.., tusb.., lwip/.., cyw43..).
#       So a drv_*.c that includes subsystemN.h plus an SDK header fails.
#   R2  Blocking SDK waits in tasks: sleep_ms/us/until, busy_wait*, mutex_enter_*,
#       sem_acquire*, pio_sm_get_blocking, pio_sm_put_blocking. Allowed only in
#       src/app/main.c on a line marked "RULE-ALLOW: pre-kernel".
#   R3a stdio calls (printf, puts, putchar, ...). Allowed only in the debug drain
#       file, or src/app/main.c lines marked "RULE-ALLOW: pre-kernel".
#   R3b T-Monitor console calls (tm_printf, tm_putstring, ...). Allowed only in the
#       port boot/fault files, or src/rtos/port/ lines marked "RULE-ALLOW: boot-console".
#   R4  Any reference to libbsp_pwm, unless the line is marked
#       "RULE-ALLOW: exclusion-note".
#   R5  Core 1 / SMP: multicore_launch_core1* calls, or pico_multicore in CMake.
#
# Scans every C/C++/asm and CMake file in the tree (src/, app_program/, config/,
# cmake/, tools/, ...), so a new directory is checked automatically. Skips only
# the top-level external/, build*/, tests/,
# docs/, .git/, .agents/, .vscode/, ProjectTemplate/ (course template, kept
# tracked as reference, never built), testing/ and videos/ (records only).
# Limits: line-based. Calls split across lines or hidden behind macros are missed.

cmake_minimum_required(VERSION 3.13)

# --- Allow-list (Batch B file names, relative to the repo root) --------------
set(RULES_STDIO_ALLOWED_FILES   "src/debug/dbg_drain.c")
set(RULES_TM_ALLOWED_FILES      "src/rtos/port/port_boot.c" "src/rtos/port/port_fault.c")
set(RULES_PREKERNEL_FILE        "src/app/main.c")
set(RULES_PORT_DIR_REGEX        "^src/rtos/port/")
set(RULES_MARK_PREKERNEL        "RULE-ALLOW: pre-kernel")
set(RULES_MARK_BOOTCONSOLE      "RULE-ALLOW: boot-console")
set(RULES_MARK_EXCLUSION        "RULE-ALLOW: exclusion-note")

# Scan scope: the WHOLE tree (fail closed). Only these top-level entries are
# skipped: vendored code, build trees, test fixtures/host tests, docs, VCS,
# agent notes, the course template (reference only) and test/video records.
set(RULES_EXCLUDE_TOP_REGEX "^(external|build[^/]*|tests|docs|\\.git|\\.agents|\\.vscode|ProjectTemplate|testing|videos)$")

if(DEFINED RULES_ROOT)
    get_filename_component(_root "${RULES_ROOT}" ABSOLUTE)
else()
    get_filename_component(_root "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
endif()

# --- Regexes (CMake regex: no \b or \s) --------------------------------------
set(_B "(^|[^A-Za-z0-9_])")
set(_RE_KINC "^[ \t]*#[ \t]*include[ \t]*[<\"](tk/|tm/|bsp/|config/mtk3/|sys/(inittask|knldef|machine|profile|queue|sysdef)\\.h|sys/sysdepend/)")
# Our kernel-side project headers (they include <tk/tkernel.h>), with any
# relative path: "app.h", "../app.h", "../test_report.h", "subsystem2.h",
# "../subsystem2/subsystem2.h".
set(_RE_KINC_PROJ "^[ \t]*#[ \t]*include[ \t]*\"([^\"]*/)?(app|test_report|subsystem[0-9]+)\\.h\"")
set(_RE_SINC "^[ \t]*#[ \t]*include[ \t]*[<\"](pico/|pico\\.h|hardware/|boards/|tusb|lwip/|cyw43)")
set(_RE_R2   "${_B}(sleep_(ms|us|until)|busy_wait[A-Za-z0-9_]*|mutex_enter_[A-Za-z0-9_]*|sem_acquire[A-Za-z0-9_]*|pio_sm_get_blocking|pio_sm_put_blocking)[ \t]*\\(")
set(_RE_R3A  "${_B}(printf|vprintf|puts|putchar|putchar_raw|puts_raw|fputs|fputc|fprintf|getchar|getchar_timeout_us|stdio_flush)[ \t]*\\(")
set(_RE_R3B  "${_B}(tm_printf|tm_putstring|tm_putchar|tm_sprintf)[ \t]*\\(")
set(_RE_R5   "${_B}multicore_launch_core1[A-Za-z0-9_]*[ \t]*\\(")
set(_RE_BLOCK "/\\*([^*]|\\*+[^*/])*\\*+/")

set(_violations 0)
set(_files_scanned 0)

# Collect files matching the given name patterns from the whole tree, skipping
# excluded top-level entries (never descends into them, so build/ stays cheap).
function(_rules_collect _out)
    set(_res "")
    file(GLOB _top LIST_DIRECTORIES true RELATIVE "${_root}" "${_root}/*")
    foreach(_e IN LISTS _top)
        if(_e MATCHES "${RULES_EXCLUDE_TOP_REGEX}")
            continue()
        endif()
        foreach(_pat IN LISTS ARGN)
            if(IS_DIRECTORY "${_root}/${_e}")
                file(GLOB_RECURSE _found "${_root}/${_e}/${_pat}")
                list(APPEND _res ${_found})
            elseif(_e MATCHES "${_pat_re_${_pat}}")
                list(APPEND _res "${_root}/${_e}")
            endif()
        endforeach()
    endforeach()
    if(_res)
        list(REMOVE_DUPLICATES _res)
        list(SORT _res)
    endif()
    set(${_out} "${_res}" PARENT_SCOPE)
endfunction()

# Regex form of each glob pattern, for files directly in the root.
set("_pat_re_*.c"            "\\.c$")
set("_pat_re_*.h"            "\\.h$")
set("_pat_re_*.S"            "\\.S$")
set("_pat_re_*.s"            "\\.s$")
set("_pat_re_*.cpp"          "\\.cpp$")
set("_pat_re_*.hpp"          "\\.hpp$")
set("_pat_re_*.cmake"        "\\.cmake$")
set("_pat_re_CMakeLists.txt" "^CMakeLists\\.txt$")

# Read a file into a list of lines (one list element per line, never empty).
# Every element is prefixed with "L" so empty lines are kept and numbering stays right.
macro(_rules_read_lines _path _out)
    file(READ "${_path}" _txt)
    string(REPLACE "\r" "" _txt "${_txt}")
    string(REPLACE "\\" "_" _txt "${_txt}")
    string(REPLACE ";" "_" _txt "${_txt}")
    string(REPLACE "[" "_" _txt "${_txt}")
    string(REPLACE "]" "_" _txt "${_txt}")
    string(REPLACE "\n" ";L" _txt "${_txt}")
    set(${_out} "L${_txt}")
endmacro()

# Function (not macro) so line text is never re-evaluated as CMake code.
function(_rules_report rel n rule msg raw)
    string(SUBSTRING "${raw}" 1 -1 shown)
    string(STRIP "${shown}" shown)
    message("${rel}:${n}: [${rule}] ${msg}: ${shown}")
    math(EXPR v "${_violations} + 1")
    set(_violations ${v} PARENT_SCOPE)
endfunction()

# --- C/C++/asm sources -------------------------------------------------------
_rules_collect(_sources "*.c" "*.h" "*.S" "*.s" "*.cpp" "*.hpp")

foreach(_f IN LISTS _sources)
    file(RELATIVE_PATH _rel "${_root}" "${_f}")
    math(EXPR _files_scanned "${_files_scanned} + 1")
    _rules_read_lines("${_f}" _lines)

    set(_in_block FALSE)
    set(_n 0)
    set(_k_line 0)
    set(_s_line 0)
    set(_k_raw "")
    set(_s_raw "")

    list(FIND RULES_STDIO_ALLOWED_FILES "${_rel}" _stdio_ok)
    list(FIND RULES_TM_ALLOWED_FILES "${_rel}" _tm_ok)
    set(_is_port FALSE)
    if(_rel MATCHES "${RULES_PORT_DIR_REGEX}")
        set(_is_port TRUE)
    endif()

    foreach(_raw IN LISTS _lines)
        math(EXPR _n "${_n} + 1")
        string(SUBSTRING "${_raw}" 1 -1 _code)

        # Strip comments (state carried across lines for /* ... */).
        if(_in_block)
            string(FIND "${_code}" "*/" _p)
            if(_p EQUAL -1)
                set(_code "")
            else()
                math(EXPR _p "${_p} + 2")
                string(SUBSTRING "${_code}" ${_p} -1 _code)
                set(_in_block FALSE)
            endif()
        endif()
        string(REGEX REPLACE "${_RE_BLOCK}" " " _code "${_code}")
        string(FIND "${_code}" "/*" _pb)
        string(FIND "${_code}" "//" _pl)
        if(NOT _pl EQUAL -1 AND (_pb EQUAL -1 OR _pl LESS _pb))
            string(SUBSTRING "${_code}" 0 ${_pl} _code)
        elseif(NOT _pb EQUAL -1)
            string(SUBSTRING "${_code}" 0 ${_pb} _code)
            set(_in_block TRUE)
        endif()

        # R1: record the first kernel and first SDK include.
        if((_code MATCHES "${_RE_KINC}" OR _code MATCHES "${_RE_KINC_PROJ}") AND _k_line EQUAL 0)
            set(_k_line ${_n})
            set(_k_raw "${_raw}")
        endif()
        if(_code MATCHES "${_RE_SINC}" AND _s_line EQUAL 0)
            set(_s_line ${_n})
            set(_s_raw "${_raw}")
        endif()

        set(_prekernel_ok FALSE)
        if(_rel STREQUAL RULES_PREKERNEL_FILE)
            string(FIND "${_raw}" "${RULES_MARK_PREKERNEL}" _m)
            if(NOT _m EQUAL -1)
                set(_prekernel_ok TRUE)
            endif()
        endif()

        # R2
        if(_code MATCHES "${_RE_R2}" AND NOT _prekernel_ok)
            _rules_report("${_rel}" ${_n} "R2" "blocking SDK wait (use tk_dly_tsk)" "${_raw}")
        endif()

        # R3a
        if(_code MATCHES "${_RE_R3A}" AND _stdio_ok EQUAL -1 AND NOT _prekernel_ok)
            _rules_report("${_rel}" ${_n} "R3a" "stdio call outside the debug drain (use dbg_)" "${_raw}")
        endif()

        # R3b
        if(_code MATCHES "${_RE_R3B}" AND _tm_ok EQUAL -1)
            set(_bc_ok FALSE)
            if(_is_port)
                string(FIND "${_raw}" "${RULES_MARK_BOOTCONSOLE}" _m)
                if(NOT _m EQUAL -1)
                    set(_bc_ok TRUE)
                endif()
            endif()
            if(NOT _bc_ok)
                _rules_report("${_rel}" ${_n} "R3b" "tm_* console call outside port boot/fault files" "${_raw}")
            endif()
        endif()

        # R4 (raw line)
        string(FIND "${_raw}" "libbsp_pwm" _m4)
        if(NOT _m4 EQUAL -1)
            string(FIND "${_raw}" "${RULES_MARK_EXCLUSION}" _mx)
            if(_mx EQUAL -1)
                _rules_report("${_rel}" ${_n} "R4" "libbsp_pwm must never be referenced" "${_raw}")
            endif()
        endif()

        # R5
        if(_code MATCHES "${_RE_R5}")
            _rules_report("${_rel}" ${_n} "R5" "core 1 must stay unused (no SMP)" "${_raw}")
        endif()
    endforeach()

    if(NOT _k_line EQUAL 0 AND NOT _s_line EQUAL 0)
        if(_k_line GREATER _s_line)
            _rules_report("${_rel}" ${_k_line} "R1" "file includes both kernel and SDK headers" "${_k_raw}")
        else()
            _rules_report("${_rel}" ${_s_line} "R1" "file includes both kernel and SDK headers" "${_s_raw}")
        endif()
    endif()
endforeach()

# --- CMake files (R4, R5 only) -----------------------------------------------
_rules_collect(_cmake_files "CMakeLists.txt" "*.cmake")
list(FILTER _cmake_files EXCLUDE REGEX "/cmake/check_rules[^/]*\\.cmake$")

foreach(_f IN LISTS _cmake_files)
    file(RELATIVE_PATH _rel "${_root}" "${_f}")
    math(EXPR _files_scanned "${_files_scanned} + 1")
    _rules_read_lines("${_f}" _lines)
    set(_n 0)
    foreach(_raw IN LISTS _lines)
        math(EXPR _n "${_n} + 1")
        string(FIND "${_raw}" "libbsp_pwm" _m4)
        if(NOT _m4 EQUAL -1)
            string(FIND "${_raw}" "${RULES_MARK_EXCLUSION}" _mx)
            if(_mx EQUAL -1)
                _rules_report("${_rel}" ${_n} "R4" "libbsp_pwm must never be referenced" "${_raw}")
            endif()
        endif()
        string(FIND "${_raw}" "pico_multicore" _m5)
        if(NOT _m5 EQUAL -1)
            _rules_report("${_rel}" ${_n} "R5" "core 1 must stay unused (no pico_multicore)" "${_raw}")
        endif()
    endforeach()
endforeach()

if(_violations GREATER 0)
    message(FATAL_ERROR "check_rules: ${_violations} violation(s)")
else()
    message(STATUS "check_rules: PASS (${_files_scanned} files scanned)")
endif()
