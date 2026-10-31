# MinGW-w64 toolchain for cross-building the plugin on Linux/WSL.
#
#   cmake -S . -B build.mingw -DCMAKE_TOOLCHAIN_FILE=mingw.cmake ...
#
# build-mingw.sh passes this file and resolves the SDK for you.

set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

# Prefer the -posix flavours: libstdc++ built for the win32 thread model has no
# std::mutex/std::thread, and plugin.cpp locks the display snapshot. winlibs and
# other single-flavour distributions only ship the plain names, so fall back to
# those instead of failing when the -posix aliases are missing.
find_program(MINGW_CXX
    NAMES "$ENV{MINGW_CXX}" x86_64-w64-mingw32-g++-posix x86_64-w64-mingw32-g++)
find_program(MINGW_CC
    NAMES "$ENV{MINGW_CC}" x86_64-w64-mingw32-gcc-posix x86_64-w64-mingw32-gcc)
find_program(MINGW_RC NAMES x86_64-w64-mingw32-windres)

foreach(_tool MINGW_CXX MINGW_CC MINGW_RC)
    if(NOT ${_tool})
        message(FATAL_ERROR
            "MinGW-w64 toolchain not found: ${_tool}. Install "
            "g++-mingw-w64-x86-64-posix (Debian/Ubuntu) or point MINGW_CXX, "
            "MINGW_CC and MINGW_RC at the toolchain binaries.")
    endif()
endforeach()
unset(_tool)

set(CMAKE_CXX_COMPILER "${MINGW_CXX}")
set(CMAKE_C_COMPILER "${MINGW_CC}")
set(CMAKE_RC_COMPILER "${MINGW_RC}")

# plugin.cpp calls GetTickCount64, which mingw-w64 only declares for Vista and
# newer; the default target is older than the Windows versions Anomaly supports.
set(CMAKE_CXX_FLAGS_INIT "-D_WIN32_WINNT=0x0601")
