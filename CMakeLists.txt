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
            src/view_mode_cycle.cpp
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

# Acceptance-only: `-DNFS2015_VIEWMODE_TRACE=ON` adds one log line per game tick
# with the controller tag, `view_active`, `chase_wanted` and the camera-guard veto
# counter. It exists so a transition can be checked frame by frame (plan TG-4);
# release packages are built without it.
option(NFS2015_VIEWMODE_TRACE "Log the view-mode state on every game tick" OFF)

target_compile_definitions(nfs2015_chase_cam PRIVATE
    UNICODE _UNICODE WIN32_LEAN_AND_MEAN NOMINMAX)
if(NFS2015_VIEWMODE_TRACE)
    target_compile_definitions(nfs2015_chase_cam PRIVATE NFS2015_VIEWMODE_TRACE)
endif()

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

    # PLAN-settings-ui-groups.md task group 1. This fixture links the real
    # settings translation unit, so the group -> effective value mapping is
    # exercised without a game process; the host services are only pointers in
    # Context and none of them is called.
    add_executable(nfs2015_settings_groups_test
        tests/settings_groups_test.cpp
        src/settings.cpp
        src/sdk_support.cpp)
    target_include_directories(nfs2015_settings_groups_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_link_libraries(nfs2015_settings_groups_test PRIVATE Anomaly::sdk)
    target_compile_definitions(nfs2015_settings_groups_test PRIVATE
        UNICODE _UNICODE WIN32_LEAN_AND_MEAN NOMINMAX)
    if(MSVC)
        target_compile_options(nfs2015_settings_groups_test PRIVATE
            /W4 /permissive- /EHsc /utf-8)
    elseif(MINGW)
        target_link_options(nfs2015_settings_groups_test PRIVATE
            -static -static-libgcc -static-libstdc++)
    endif()
    add_test(NAME nfs2015_settings_groups_model
             COMMAND nfs2015_settings_groups_test)

    # PLAN-settings-ui-groups.md task group 2. This fixture drives the real
    # persistence layer through a fake config service and a fake JSON service,
    # so a document round trip and the legacy inference are pinned without a
    # host process.
    add_executable(nfs2015_settings_persistence_test
        tests/settings_persistence_test.cpp
        src/settings.cpp
        src/sdk_support.cpp)
    target_include_directories(nfs2015_settings_persistence_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_link_libraries(nfs2015_settings_persistence_test
        PRIVATE Anomaly::sdk)
    target_compile_definitions(nfs2015_settings_persistence_test PRIVATE
        UNICODE _UNICODE WIN32_LEAN_AND_MEAN NOMINMAX)
    if(MSVC)
        target_compile_options(nfs2015_settings_persistence_test PRIVATE
            /W4 /permissive- /EHsc /utf-8)
    elseif(MINGW)
        target_link_options(nfs2015_settings_persistence_test PRIVATE
            -static -static-libgcc -static-libstdc++)
    endif()
    add_test(NAME nfs2015_settings_persistence_model
             COMMAND nfs2015_settings_persistence_test)

    # PLAN-settings-ui-groups.md task group 3. The panel is drawn against a
    # recording fake UI service, so the mode rows, the detail visibility and the
    # widget ids are checked without a host.
    add_executable(nfs2015_settings_panel_test
        tests/settings_panel_test.cpp
        src/ui.cpp
        src/settings.cpp
        src/sdk_support.cpp)
    target_include_directories(nfs2015_settings_panel_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_link_libraries(nfs2015_settings_panel_test
        PRIVATE Anomaly::sdk)
    target_compile_definitions(nfs2015_settings_panel_test PRIVATE
        UNICODE _UNICODE WIN32_LEAN_AND_MEAN NOMINMAX)
    if(MSVC)
        target_compile_options(nfs2015_settings_panel_test PRIVATE
            /W4 /permissive- /EHsc /utf-8)
    elseif(MINGW)
        target_link_options(nfs2015_settings_panel_test PRIVATE
            -static -static-libgcc -static-libstdc++)
    endif()
    add_test(NAME nfs2015_settings_panel
             COMMAND nfs2015_settings_panel_test)
endif()
