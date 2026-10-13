# cmake/warnings.cmake
#
# robocar_apply_warnings(<source>...)
#
# Applies -Wall -Wextra -Werror to OUR source files only. It is applied per
# source file, not per target, because SDK INTERFACE-library sources are
# compiled inside the robocar target and must not get -Werror. Never use this
# on the mtk3 library or on SDK sources.
#
# GCC 10.3 (Pico SDK 1.5.1 installer): do not add -fdiagnostics-plain-output.

set(ROBOCAR_WARNING_FLAGS -Wall -Wextra -Werror)

function(robocar_apply_warnings)
    if(ARGN)
        set_source_files_properties(${ARGN} PROPERTIES COMPILE_OPTIONS "${ROBOCAR_WARNING_FLAGS}")
    endif()
endfunction()
