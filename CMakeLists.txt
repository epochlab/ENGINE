cmake_minimum_required(VERSION 3.21)
project(Pathtracer LANGUAGES CXX OBJCXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Single-config generators (Makefiles/Ninja) default to an unoptimized build if CMAKE_BUILD_TYPE is unset: silently ~40x slower (measured: a 5M-triangle glTF load went 1.1s Release vs 47s+ unoptimized).
# Multi-config generators (Xcode) ignore this and use CMAKE_CONFIGURATION_TYPES instead, hence the guard.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(OpenGL REQUIRED)
find_package(glfw3 3.5 REQUIRED)
find_package(GLEW REQUIRED)
# Homebrew's glew config links OpenGL.framework by absolute path into the SDK glew was built with; a foreign SDK there enters the -F search path and shadows this build's own system frameworks as user headers.
set_property(TARGET GLEW::GLEW PROPERTY INTERFACE_LINK_LIBRARIES OpenGL::GL)
find_package(glm REQUIRED)
find_package(Imath REQUIRED)
find_package(OpenEXR REQUIRED)
find_package(OpenColorIO REQUIRED)
find_package(embree 4 CONFIG REQUIRED)
# render_beauty's PNG encode only. Not a new third-party dependency: OpenEXR already links zlib transitively and the platform ships it -- naming it here just makes the direct use explicit.
find_package(ZLIB REQUIRED)

# Static-analysis gate: -Wall -Wextra -Werror plus clang-tidy/cppcheck, applied to every build, not just CI. clang-tidy is wired below as a per-compile gate (non-fatal: it surfaces findings every build without turning pre-existing debt into a hard build failure); cppcheck is a separate `cmake --build build --target cppcheck` target. Both are no-ops (not required) if not installed, so a fresh checkout without them still configures and builds.
find_program(CLANG_TIDY_EXE NAMES clang-tidy)
if(CLANG_TIDY_EXE)
    # Homebrew's clang-tidy doesn't share Xcode clang's default sysroot, so without this it fails to find the standard library headers entirely rather than just misreporting one warning.
    execute_process(COMMAND xcrun --show-sdk-path
                     OUTPUT_VARIABLE MACOS_SDK_PATH
                     OUTPUT_STRIP_TRAILING_WHITESPACE)
    set(CLANG_TIDY_COMMAND ${CLANG_TIDY_EXE}
        --extra-arg=-isysroot --extra-arg=${MACOS_SDK_PATH})
endif()

# Every first-party target's compile flags in one place. NATIVE/IPO are opt-in rather than default because the
# codegen tools (albedo_table, metal_fit) must reproduce their committed output bit-for-bit on any machine, which
# -march=native and FMA contraction under IPO would break. TIDY is opt-in because only the three shipping targets
# are gated; the tools are checked by cppcheck and ctest instead.
function(pathtracer_target_defaults target)
    cmake_parse_arguments(ARG "NATIVE;IPO;TIDY" "" "" ${ARGN})
    target_compile_options(${target} PRIVATE -Wall -Wextra -Werror)
    if(ARG_NATIVE)
        target_compile_options(${target} PRIVATE -march=native)
    endif()
    if(ARG_IPO AND IPO_SUPPORTED)
        set_target_properties(${target} PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
    endif()
    if(ARG_TIDY AND CLANG_TIDY_EXE)
        set_target_properties(${target} PROPERTIES CXX_CLANG_TIDY "${CLANG_TIDY_COMMAND}")
    endif()
endfunction()

include(CheckIPOSupported)
check_ipo_supported(RESULT IPO_SUPPORTED)

# Build provenance for the startup spec block: a timing number without the machine and build it was measured on is not a measurement.
# Build-time, not configure-time: the SHA keys benchmark-log records, so a commit made without re-running cmake must not be attributed to its parent. Costs two git spawns per build; the header is rewritten only when the SHA changes.
find_package(Git QUIET)
set(PATHTRACER_GENERATED_DIR ${CMAKE_BINARY_DIR}/generated)
add_custom_target(pathtracer_git_sha ALL
    COMMAND ${CMAKE_COMMAND} -DSOURCE_DIR=${CMAKE_SOURCE_DIR} -DOUTPUT=${PATHTRACER_GENERATED_DIR}/pathtracer_git_sha.h
            -DGIT_EXECUTABLE=${GIT_EXECUTABLE} -P ${CMAKE_SOURCE_DIR}/cmake/GitSha.cmake
    BYPRODUCTS ${PATHTRACER_GENERATED_DIR}/pathtracer_git_sha.h
    COMMENT "Stamping git SHA")
# Compile-time half of BuildInfo, for every target compiling host_info.cpp. Every such target builds with -march=native and IPO-when-supported, which is what these report; a target that differs must not use this list.
# $<CONFIG>, not CMAKE_BUILD_TYPE: correct under both the single-config generators the guard at the top of this file defaults and the multi-config Xcode generator that ignores it.
set(PATHTRACER_PROVENANCE_DEFINITIONS PATHTRACER_BUILD_TYPE="$<CONFIG>" PATHTRACER_MARCH="native" PATHTRACER_IPO=$<BOOL:${IPO_SUPPORTED}>)

find_program(CPPCHECK_EXE NAMES cppcheck)
if(CPPCHECK_EXE)
    add_custom_target(cppcheck
        COMMAND ${CPPCHECK_EXE} --enable=warning,style,performance --std=c++20
                --suppress=missingInclude
                --suppressions-list=${CMAKE_SOURCE_DIR}/.cppcheck-suppressions
                -I ${CMAKE_SOURCE_DIR}/include
                ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/tools
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "cppcheck over src/ and tools/")
endif()

# The comment budget of notes/architect.md section 7, which had no gate on the C++ side: python/pyproject.toml's
# ruff line-length enforced it for Python only, so the C++ tree drifted to 12.3% comments and a 3143-column line.
# Reporting-only for now; it becomes a ctest gate once the tree is clean.
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
    add_custom_target(comment_lint
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/comment_lint.py --report
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "comment budget over src/, include/ and tools/")
endif()

# Opt-in (default off, zero effect on a normal build): the pathtracer parses external glTF/JSON/EXR files, where memory-safety tooling earns its keep. `cmake -B build -DPATHTRACER_SANITIZE=ON`.
option(PATHTRACER_SANITIZE "Build with AddressSanitizer + UndefinedBehaviorSanitizer" OFF)
if(PATHTRACER_SANITIZE)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()

set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/third_party/imgui)
if(NOT EXISTS ${IMGUI_DIR}/imgui.cpp)
    message(FATAL_ERROR "third_party/imgui is empty, run: git submodule update --init")
endif()

set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp)

set(CGLTF_DIR ${CMAKE_SOURCE_DIR}/third_party/cgltf)
set(JSON_DIR ${CMAKE_SOURCE_DIR}/third_party)

# Vendored code, not ours to fix: imgui_impl_glfw.cpp pulls in Apple's ApplicationServices.h via GLFW's native-handle header, which warns against itself; cgltf_impl.cpp is C99 with implicit conversions that trip -Wall/-Wextra.
# Built as OBJECT libraries rather than per-source overrides: CXX_CLANG_TIDY is target-level-only in CMake, so keeping clang-tidy off vendored sources requires a target that never sets the property.
# Two libraries, not one: cgltf is needed by every headless consumer and imgui by none of them, so bundling them made glfw and all of imgui a link dependency of render_beauty, which never opens a window.
add_library(vendored_cgltf OBJECT src/scene/cgltf_impl.cpp)
target_include_directories(vendored_cgltf PRIVATE ${CGLTF_DIR})
target_compile_options(vendored_cgltf PRIVATE -w)

add_library(vendored_imgui OBJECT ${IMGUI_SOURCES})
target_include_directories(vendored_imgui PRIVATE ${IMGUI_DIR} ${IMGUI_DIR}/backends)
target_link_libraries(vendored_imgui PRIVATE glfw)
target_compile_options(vendored_imgui PRIVATE -w)

# The renderer proper: every source that does not touch GL/GLFW/imgui, listed once and linked by everything.
# The membership rule is exactly "does this file include a GL, GLFW or imgui header" -- nothing here can open a window, which is what lets a foreign runtime (tools/, and the C ABI in src/api/) link it.
# cie.cpp is deliberately absent: it belongs to metal_fit_core, which is built without -march=native/IPO so its committed output reproduces bit-for-bit on any machine, and having it here too would be a duplicate symbol.
# -march=native/IPO match what the GUI target used to build these sources with, so the library is the same codegen that shipped, not a differently-optimized copy of the same math.
add_library(pathtracer_core STATIC
    src/config/profile_config.cpp
    src/config/scene_config.cpp
    src/debug/aov.cpp
    src/debug/aov_filters.cpp
    src/debug/bench_log.cpp
    src/debug/frame_stats.cpp
    src/debug/host_info.cpp
    src/debug/memory_tracker.cpp
    src/debug/perf_dashboard.cpp
    src/debug/power_spectrum.cpp
    src/gfx/hdr_image.cpp
    src/gfx/ocio_cpu_transform.cpp
    src/scene/bsdf.cpp
    src/scene/camera.cpp
    src/scene/embree_accel.cpp
    src/scene/environment_map.cpp
    src/scene/gbuffer_shading.cpp
    src/scene/gltf_loader.cpp
    src/scene/light.cpp
    src/scene/material.cpp
    src/scene/material_binding.cpp
    src/scene/path_trace_driver.cpp
    src/scene/path_tracer.cpp
    src/scene/rasterizer.cpp
    src/scene/ray_types.cpp
    src/scene/sampler.cpp
    src/scene/shading_scene.cpp
    src/scene/thread_pool.cpp
    src/api/headless_renderer.cpp
    $<TARGET_OBJECTS:vendored_cgltf>)
# include/ and the vendored json headers are PUBLIC because every consumer includes pathtracer/... and several parse JSON; cgltf and the generated SHA header are implementation details of gltf_loader.cpp and host_info.cpp respectively.
target_include_directories(pathtracer_core PUBLIC include ${JSON_DIR})
target_include_directories(pathtracer_core PRIVATE ${CGLTF_DIR} ${PATHTRACER_GENERATED_DIR})
add_dependencies(pathtracer_core pathtracer_git_sha)
target_compile_definitions(pathtracer_core PRIVATE ${PATHTRACER_PROVENANCE_DEFINITIONS})
pathtracer_target_defaults(pathtracer_core NATIVE IPO TIDY)
target_link_libraries(pathtracer_core PUBLIC glm::glm embree OpenEXR::OpenEXR Imath::Imath
                       OpenColorIO::OpenColorIO ZLIB::ZLIB)

# The C ABI a foreign runtime binds to (python/pathtracer loads this via ctypes). Shared, not static: it is dlopen'd, not linked.
# Deliberately the only target here that is not an executable or an object library -- everything it exposes is declared in include/pathtracer/api/pathtracer_c.h and implemented over pathtracer_core.
add_library(pathtracer_c SHARED src/api/pathtracer_c.cpp)
target_compile_definitions(pathtracer_c PRIVATE ASSET_ROOT_DIR="${CMAKE_SOURCE_DIR}/assets")
pathtracer_target_defaults(pathtracer_c NATIVE IPO TIDY)
target_link_libraries(pathtracer_c PRIVATE pathtracer_core)

# The GUI application: main.cpp plus exactly the sources that touch GL/GLFW/imgui. Everything else it needs comes from pathtracer_core.
add_executable(pathtracer
    src/main.cpp
    src/platform/window.cpp
    src/platform/display_link.mm
    src/gfx/gl_debug.cpp
    src/gfx/shader_program.cpp
    src/gfx/texture.cpp
    src/gfx/post_process_pass.cpp
    src/gfx/ocio_display_transform.cpp
    src/scene/debug_camera_controller.cpp
    src/debug/gpu_timer.cpp
    src/debug/histogram.cpp
    src/debug/hud_overlay.cpp
    src/debug/system_info.cpp
    src/debug/spec_report.cpp
    $<TARGET_OBJECTS:vendored_imgui>)
target_include_directories(pathtracer PRIVATE ${IMGUI_DIR} ${IMGUI_DIR}/backends)
target_compile_definitions(pathtracer PRIVATE
    GL_SILENCE_DEPRECATION
    ASSET_ROOT_DIR="${CMAKE_SOURCE_DIR}/assets")
# IPO is a free codegen speedup for the path tracer's hot loop (ray traversal, BSDF eval), no behaviour or quality change.
pathtracer_target_defaults(pathtracer NATIVE IPO TIDY)
target_link_libraries(pathtracer PRIVATE
    pathtracer_core glfw GLEW::GLEW OpenGL::GL
    "-framework AppKit" "-framework QuartzCore")
set_source_files_properties(src/platform/display_link.mm PROPERTIES COMPILE_OPTIONS -fobjc-arc)

# Asset/codegen CLIs. These deliberately do NOT link pathtracer_core: test_pattern and downsample need
# nothing but OpenEXR, gltf_tangent nothing but glm, and albedo_table/metal_fit are built without
# -march=native/IPO so their committed output reproduces bit-for-bit on any machine (see below).
add_executable(test_pattern tools/test_pattern.cpp)
target_compile_definitions(test_pattern PRIVATE
    ASSET_ROOT_DIR="${CMAKE_SOURCE_DIR}/assets")
pathtracer_target_defaults(test_pattern)
target_link_libraries(test_pattern PRIVATE OpenEXR::OpenEXR Imath::Imath)

add_executable(downsample tools/downsample.cpp)
pathtracer_target_defaults(downsample)
target_link_libraries(downsample PRIVATE OpenEXR::OpenEXR Imath::Imath)

# One-off asset-prep for Houdini glTF exports: synthesizes a TANGENT accessor for any primitive
# that lacks one, in place. Every other attribute (POSITION/NORMAL/TEXCOORD_0/COLOR_0/...), every
# material, and every other primitive is patched, not rebuilt -- gltf_loader.cpp's own default-texture
# substitution (and its acceptance of a primitive with no material at all) handles the rest.
add_executable(gltf_tangent tools/gltf_tangent.cpp)
target_include_directories(gltf_tangent PRIVATE ${JSON_DIR})
pathtracer_target_defaults(gltf_tangent)
target_link_libraries(gltf_tangent PRIVATE glm::glm)

# Offline generator for src/scene/blue_noise_mask.inc, the void-and-cluster dither array sampler.cpp bakes in. Grouped
# with gltf_tangent rather than with the validate tools below: it produces a committed artifact, it does not check one,
# so it stays out of the ctest loop. -march=native/IPO because it is convolution-bound and only ever run by hand.
add_executable(bluenoise_mask tools/bluenoise_mask.cpp)
pathtracer_target_defaults(bluenoise_mask NATIVE IPO)
target_link_libraries(bluenoise_mask PRIVATE pathtracer_core)

# Offline generator for src/scene/albedo_table.inc, the Kulla-Conty energy tables bsdf.cpp bakes in. Grouped with
# bluenoise_mask for the same reason: it produces a committed artifact, it does not check one, so it stays out of the
# ctest loop. Neither -march=native nor IPO, unlike every other tool here -- the output is committed source and must
# reproduce bit-for-bit on any machine, and FMA contraction is free to differ; the bake runs in seconds regardless.
# Links no library for that reason: pathtracer_core is built -march=native, which is exactly what this must avoid.
add_executable(albedo_table tools/albedo_table.cpp)
target_include_directories(albedo_table PRIVATE include)
pathtracer_target_defaults(albedo_table)
target_link_libraries(albedo_table PRIVATE glm::glm)

# Measured conductor (lambda, n, k) to the reflectivity/edgeTint a material JSON carries (tools/metal_fit.h). Shared as one object library by the metal_fit authoring CLI and colour_validate's drift check, so both run the same codegen; no -march=native/IPO, for albedo_table's reproducibility reason.
add_library(metal_fit_core OBJECT tools/metal_fit.cpp src/scene/cie.cpp)
target_include_directories(metal_fit_core PUBLIC include tools)
pathtracer_target_defaults(metal_fit_core)
target_link_libraries(metal_fit_core PRIVATE glm::glm)

add_executable(metal_fit tools/metal_fit_main.cpp $<TARGET_OBJECTS:metal_fit_core>)
target_include_directories(metal_fit PRIVATE include tools)
pathtracer_target_defaults(metal_fit)
target_link_libraries(metal_fit PRIVATE glm::glm)

# Headless correctness checks (Phase 4): no test framework exists yet, so these follow test_pattern/downsample's standalone-CLI convention (non-zero exit on failure).
# -march=native/IPO mirror the pathtracer target's flags so these tools validate the exact codegen that ships, not a differently-optimized build of the same math.
# Shared validator harness (tools/check.h, tools/stats.h). Deliberately depends on nothing but glm: the light
# validators (sampler, embree) must not be forced to link Embree or OpenEXR just to reach a chi-square survival
# function. Built with the same flags as the tools it serves so the harness is not a different codegen from them.
add_library(pathtracer_check OBJECT tools/check.cpp)
target_include_directories(pathtracer_check PUBLIC tools)
pathtracer_target_defaults(pathtracer_check NATIVE)
target_link_libraries(pathtracer_check PRIVATE glm::glm)

# Guards the harness's own invariants -- NaN rejection and name-derived seeds -- which nothing else would notice
# regressing, because a broken assert macro reports success.
add_executable(check_selftest tools/check_selftest.cpp $<TARGET_OBJECTS:pathtracer_check>)
target_include_directories(check_selftest PRIVATE tools)
pathtracer_target_defaults(check_selftest NATIVE)
target_link_libraries(check_selftest PRIVATE glm::glm)

# Every validator below links pathtracer_core for the renderer sources it exercises, and pathtracer_check for the harness.
# What each one asserts is documented at the top of its own tools/*.cpp; only the targets that need an asset path or a
# second object library say anything extra here.
foreach(validator embree_validate bsdf_validate sampler_validate nee_validate rasterizer_validate
                  integrator_validate driver_validate io_validate display_validate)
    add_executable(${validator} tools/${validator}.cpp $<TARGET_OBJECTS:pathtracer_check>)
    target_include_directories(${validator} PRIVATE tools)
    pathtracer_target_defaults(${validator} NATIVE IPO)
    target_link_libraries(${validator} PRIVATE pathtracer_core)
endforeach()

# integrator_validate's checkMaterialBinding and io_validate's config checks resolve real assets/ files against named
# instances -- the materialOverrides path no other validator touches.
target_compile_definitions(integrator_validate PRIVATE ASSET_ROOT_DIR="${CMAKE_SOURCE_DIR}/assets")
target_compile_definitions(io_validate PRIVATE ASSET_ROOT_DIR="${CMAKE_SOURCE_DIR}/assets")

# CIE 1931 module and the measured-metal fit, checked end to end against the Fresnel the renderer shades chrome.json with.
add_executable(colour_validate tools/colour_validate.cpp
               $<TARGET_OBJECTS:metal_fit_core> $<TARGET_OBJECTS:pathtracer_check>)
target_include_directories(colour_validate PRIVATE tools)
target_compile_definitions(colour_validate PRIVATE ASSET_ROOT_DIR="${CMAKE_SOURCE_DIR}/assets"
                           TOOLS_DATA_DIR="${CMAKE_SOURCE_DIR}/tools/data")
pathtracer_target_defaults(colour_validate NATIVE IPO)
target_link_libraries(colour_validate PRIVATE pathtracer_core)

# The headless render path itself (src/api/headless_renderer.cpp) exercised through the C ABI's own contract:
# producer dispatch, per-AOV channel counts, and agreement with a direct renderPathTraced accumulation.
add_executable(api_validate tools/api_validate.cpp $<TARGET_OBJECTS:pathtracer_check>)
target_include_directories(api_validate PRIVATE tools)
target_compile_definitions(api_validate PRIVATE ASSET_ROOT_DIR="${CMAKE_SOURCE_DIR}/assets")
pathtracer_target_defaults(api_validate NATIVE IPO)
target_link_libraries(api_validate PRIVATE pathtracer_core)

# Headless before/after comparison renders (tools/render_beauty.cpp). Not a correctness gate and deliberately absent from the add_test loop below: its output is a picture for a human to look at, and a full-scene render is far slower than any validator.
add_executable(render_beauty tools/render_beauty.cpp $<TARGET_OBJECTS:pathtracer_check>)
target_include_directories(render_beauty PRIVATE tools)
target_compile_definitions(render_beauty PRIVATE ASSET_ROOT_DIR="${CMAKE_SOURCE_DIR}/assets")
pathtracer_target_defaults(render_beauty NATIVE IPO)
target_link_libraries(render_beauty PRIVATE pathtracer_core)

# Timing harness, not a correctness gate: deliberately absent from the add_test loop below (a benchmark's pass/fail is a human reading a number, and it would make ctest as slow as its worst --triangles sweep). Same flags as the validators, so it times the exact codegen that ships.
add_executable(raster_bench tools/raster_bench.cpp)
pathtracer_target_defaults(raster_bench NATIVE IPO)
target_link_libraries(raster_bench PRIVATE pathtracer_core)

# Benchmark-log analysis and randomized interleaved A/B runner (tools/bench_compare.cpp). Not a correctness gate, for the same reason as raster_bench: its verdict is a confidence interval for a human to read.
add_executable(bench_compare tools/bench_compare.cpp)
target_include_directories(bench_compare PRIVATE include tools ${JSON_DIR})
pathtracer_target_defaults(bench_compare)


# All correctness tools run under `ctest --test-dir build`; previously buildable but unrun, so a regression only surfaced if someone remembered to invoke them by hand.
enable_testing()

# Concurrent checks would each build a pool sized to hardware_concurrency(), so `ctest -j` would oversubscribe by that
# factor. Each discovered test declares PROCESSORS to match, so ctest's job pool accounts for what it will actually use.
include(ProcessorCount)
ProcessorCount(PATHTRACER_HOST_PROCESSORS)
if(PATHTRACER_HOST_PROCESSORS EQUAL 0)
    set(PATHTRACER_HOST_PROCESSORS 1)
endif()
math(EXPR PATHTRACER_TEST_THREADS "${PATHTRACER_HOST_PROCESSORS} / 4")
if(PATHTRACER_TEST_THREADS LESS 1)
    set(PATHTRACER_TEST_THREADS 1)
endif()

include(${CMAKE_SOURCE_DIR}/cmake/PathtracerChecks.cmake)

# Image-level gates. Not part of a validator binary because they need the real scene, the real glTF and the real
# material stack -- what they assert is a property of the shipping pipeline, not of a synthetic fixture.
# Deliberately small and short: both are statements about the ESTIMATOR, which does not need a converged image to
# test, and a gate that takes minutes stops being run.
add_test(NAME image.determinism
         COMMAND render_beauty --scene scenes/cornell.json --width 48 --height 36 --passes 8
                 --assert-deterministic)
set_tests_properties(image.determinism PROPERTIES LABELS "image;fast;exact" TIMEOUT 300)

# A rasterizer-backed AOV runs no path-traced passes, so there is no per-pass distribution for the timing report to
# summarise. Reducing over that empty set segfaulted until the reporting learned to say so; at this size the gate is
# nearly free, and it is the only test that exercises render_beauty's non-path-traced output path end to end.
add_test(NAME image.gbuffer_aov
         COMMAND render_beauty --scene scenes/cornell.json --width 48 --height 36 --passes 4
                 --aov depth --out gbuffer_aov.png)
set_tests_properties(image.gbuffer_aov PROPERTIES LABELS "image;fast;exact" TIMEOUT 300)

add_test(NAME image.convergence
         COMMAND render_beauty --scene scenes/cornell.json --width 48 --height 36 --passes 64
                 --assert-converged)
set_tests_properties(image.convergence PROPERTIES LABELS "image;slow;statistical" TIMEOUT 900
                     PROCESSORS ${PATHTRACER_TEST_THREADS})

# Migrated to tools/check.h: one ctest entry per check, discovered from the binary itself.
pathtracer_discover_checks(check_selftest)
pathtracer_discover_checks(sampler_validate)
pathtracer_discover_checks(embree_validate)
pathtracer_discover_checks(rasterizer_validate)
pathtracer_discover_checks(nee_validate)
pathtracer_discover_checks(integrator_validate)
pathtracer_discover_checks(bsdf_validate)
pathtracer_discover_checks(driver_validate)
pathtracer_discover_checks(io_validate)
pathtracer_discover_checks(display_validate)
pathtracer_discover_checks(colour_validate)
pathtracer_discover_checks(api_validate)

