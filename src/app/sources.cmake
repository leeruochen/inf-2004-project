# src/app/sources.cmake: SDK main() (the firmware entry point).
# ROBOCAR_SOURCES = SDK-side files (SDK headers only).
# Its existence switches CMake from compile-check to the full firmware link
# (CMakeLists.txt, "src/app/main.c").
list(APPEND ROBOCAR_SOURCES "${CMAKE_CURRENT_LIST_DIR}/main.c")
