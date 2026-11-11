cmake_minimum_required(VERSION 3.22)

project(Nfs2015ChaseCam LANGUAGES CXX)

# Standalone third-party plugin project. It is independent of the Anomaly
# source tree and consumes the installed Anomaly SDK only:
#
#   cmake --install <anomaly-build> --prefix <sdk> --component SDK
#   cmake -S . -B build -DAnomalySDK_DIR=<sdk>/lib/cmake/AnomalySDK
#   cmake --build build --config RelWithDebInfo
#
# The resulting package is written to build/package/robineatcorn.nfs2015-chase-cam.

if(NOT WIN32 OR NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "The Anomaly plugin ABI targets Windows x64")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(AnomalySDK CONFIG REQUIRED)

anomaly_add_plugin(nfs2015_chase_cam
    SOURCES plugin.cpp
            src/globals.cpp
            src/sdk_support.cpp
            src/game_tick.cpp
            src/presenter.cpp
            src/display.cpp
            src/lifecycle.cpp
            src/hooks.cpp
            src/camera_detour.cpp
            src/routeb.cpp
            src/render_x_probe.cpp
            src/burnout.cpp
            src/wheel_steer.cpp
            src/vehicle.cpp
            src/ahud_probe.cpp
            src/settings.cpp
            src/ui.cpp
    MANIFEST manifest.json
    PACKAGE_NAME robineatcorn.nfs2015-chase-cam)

# Internal headers live in src/; the model headers stay in the source root
# because tests include them by relative path.
target_include_directories(nfs2015_chase_cam PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src")

# The package directory must only hold what the Runtime ships: the DLL, the
# manifest and the localized catalogs. Import libraries and PDBs are build
# products, and `anomaly-plugin pack` copies every file it finds.
set_target_properties(nfs2015_chase_cam PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib"
    PDB_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/symbols")

target_compile_definitions(nfs2015_chase_cam PRIVATE
    UNICODE _UNICODE WIN32_LEAN_AND_MEAN NOMINMAX)

if(MSVC)
    # The sources are UTF-8 without a BOM; without /utf-8 MSVC decodes them
    # with the system code page.
    target_compile_options(nfs2015_chase_cam PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:/utf-8>)
elseif(MINGW)
    # The plugin loader only accepts system DLL imports or libraries that ship
    # inside the package. Link the GCC runtime statically so the DLL does not
    # depend on libstdc++-6.dll, libgcc_s_seh-1.dll or libwinpthread-1.dll;
    # KERNEL32 and msvcrt are Windows system modules and pass the precheck.
    target_link_options(nfs2015_chase_cam PRIVATE
        -static -static-libgcc -static-libstdc++)
endif()

option(NFS2015_CHASE_CAM_BUILD_TESTS
    "Build the chase-camera model acceptance fixture" ON)

if(NFS2015_CHASE_CAM_BUILD_TESTS)
    enable_testing()
    add_executable(nfs2015_chase_cam_test tests/chase_cam_test.cpp)
    target_include_directories(nfs2015_chase_cam_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}")
    if(MSVC)
        target_compile_options(nfs2015_chase_cam_test PRIVATE
            /W4 /permissive- /EHsc /utf-8)
    elseif(MINGW)
        target_link_options(nfs2015_chase_cam_test PRIVATE
            -static -static-libgcc -static-libstdc++)
    endif()
    add_test(NAME nfs2015_chase_cam_model COMMAND nfs2015_chase_cam_test)

    add_executable(nfs2015_burnout_shot_test tests/burnout_shot_test.cpp)
    target_include_directories(nfs2015_burnout_shot_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}")
    if(MSVC)
        target_compile_options(nfs2015_burnout_shot_test PRIVATE
            /W4 /permissive- /EHsc /utf-8)
    elseif(MINGW)
        target_link_options(nfs2015_burnout_shot_test PRIVATE
            -static -static-libgcc -static-libstdc++)
    endif()
    add_test(NAME nfs2015_burnout_shot_model COMMAND nfs2015_burnout_shot_test)

    add_executable(nfs2015_wheel_steer_test tests/wheel_steer_test.cpp)
    target_include_directories(nfs2015_wheel_steer_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}")
    if(MSVC)
        target_compile_options(nfs2015_wheel_steer_test PRIVATE
            /W4 /permissive- /EHsc /utf-8)
    elseif(MINGW)
        target_link_options(nfs2015_wheel_steer_test PRIVATE
            -static -static-libgcc -static-libstdc++)
    endif()
    add_test(NAME nfs2015_wheel_steer_model COMMAND nfs2015_wheel_steer_test)
endif()
