# tools/json_sizes.cmake
#
# Byte sizes of the reference JSON baseline (tools/json_baseline/*.json)
# against the planned binary MQTT frames (ARCHITECTURE.md D10,
# docs/DATA_REPRESENTATION.md). Host-side only; needs CMake >= 3.19
# (string(JSON)). Run from the repo root with the installed CMake:
#   cmake -P tools/json_sizes.cmake
#
# The JSON files are an invented reference for size comparison, not a
# protocol: design.md defines no JSON schema. Each file is one minified line
# with no trailing newline, values at their widest printed form.

cmake_minimum_required(VERSION 3.19)

get_filename_component(_dir "${CMAKE_CURRENT_LIST_DIR}/json_baseline" ABSOLUTE)

# Binary frame = ver u8 | type u8 | len u8 | payload[len], little-endian.
# Planned payload bytes per type. Keep equal to the "Binary payload" column of
# docs/DATA_REPRESENTATION.md section 8 (24-bit timestamp + packed fields).
set(_FRAME_HEADER_BYTES 3)
set(_payload_telemetry_motion 15)
set(_payload_telemetry_sensor 10)
set(_payload_event_barcode 4)
set(_payload_event_hump 5)
set(_payload_event_obstacle 8)
set(_payload_event_fault 5)
set(_payload_heartbeat 6)
set(_payload_cmd_start 0)
set(_payload_cmd_stop 0)
set(_payload_cmd_reset 0)

file(GLOB _files "${_dir}/*.json")
list(SORT _files)
if(NOT _files)
    message(FATAL_ERROR "json_sizes: no files in ${_dir}")
endif()

set(_out "| type | JSON bytes | binary frame bytes | JSON / binary |\n|---|---|---|---|\n")
set(_sum_json 0)
set(_sum_bin 0)
foreach(_f IN LISTS _files)
    get_filename_component(_name "${_f}" NAME_WE)
    if(NOT DEFINED _payload_${_name})
        message(FATAL_ERROR "json_sizes: no planned payload size for '${_name}'")
    endif()
    file(READ "${_f}" _txt)
    string(LENGTH "${_txt}" _len)
    string(REGEX MATCH "[\r\n]$" _nl "${_txt}")
    if(_nl)
        message(FATAL_ERROR "json_sizes: ${_name}.json ends with a newline")
    endif()
    string(JSON _type ERROR_VARIABLE _err TYPE "${_txt}")
    if(_err OR NOT _type STREQUAL "OBJECT")
        message(FATAL_ERROR "json_sizes: ${_name}.json is not a valid JSON object: ${_err}")
    endif()
    math(EXPR _bin "${_FRAME_HEADER_BYTES} + ${_payload_${_name}}")
    # ratio with one decimal, integer arithmetic only
    math(EXPR _r10 "(${_len} * 10 + ${_bin} / 2) / ${_bin}")
    math(EXPR _ri "${_r10} / 10")
    math(EXPR _rf "${_r10} % 10")
    string(APPEND _out "| ${_name} | ${_len} | ${_bin} | ${_ri}.${_rf} |\n")
    math(EXPR _sum_json "${_sum_json} + ${_len}")
    math(EXPR _sum_bin "${_sum_bin} + ${_bin}")
endforeach()
math(EXPR _r10 "(${_sum_json} * 10 + ${_sum_bin} / 2) / ${_sum_bin}")
math(EXPR _ri "${_r10} / 10")
math(EXPR _rf "${_r10} % 10")
string(APPEND _out "| total | ${_sum_json} | ${_sum_bin} | ${_ri}.${_rf} |")
message("${_out}")
