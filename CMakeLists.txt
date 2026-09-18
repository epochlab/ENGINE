cmake_minimum_required(VERSION 3.21)
project(Engine LANGUAGES CXX)
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

include(CheckIPOSupported)
check_ipo_supported(RESULT IPO_SUPPORTED)

# Build provenance for the startup spec block: a timing number without the machine and build it was measured on is not a measurement.
# Build-time, not configure-time: the SHA keys benchmark-log records, so a commit made without re-running cmake must not be attributed to its parent. Costs two git spawns per build; the header is rewritten only when the SHA changes.
find_package(Git QUIET)
set(ENGINE_GENERATED_DIR ${CMAKE_BINARY_DIR}/generated)
add_custom_target(engine_git_sha ALL
    COMMAND ${CMAKE_COMMAND} -DSOURCE_DIR=${CMAKE_SOURCE_DIR} -DOUTPUT=${ENGINE_GENERATED_DIR}/engine_git_sha.h
            -DGIT_EXECUTABLE=${GIT_EXECUTABLE} -P ${CMAKE_SOURCE_DIR}/cmake/GitSha.cmake
    BYPRODUCTS ${ENGINE_GENERATED_DIR}/engine_git_sha.h
    COMMENT "Stamping git SHA")
# Compile-time half of BuildInfo, for every target compiling host_info.cpp. Every such target builds with -march=native and IPO-when-supported, which is what these report; a target that differs must not use this list.
# $<CONFIG>, not CMAKE_BUILD_TYPE: correct under both the single-config generators the guard at the top of this file defaults and the multi-config Xcode generator that ignores it.
set(ENGINE_PROVENANCE_DEFINITIONS ENGINE_BUILD_TYPE="$<CONFIG>" ENGINE_MARCH="native" ENGINE_IPO=$<BOOL:${IPO_SUPPORTED}>)

find_program(CPPCHECK_EXE NAMES cppcheck)
if(CPPCHECK_EXE)
    add_custom_target(cppcheck
        COMMAND ${CPPCHECK_EXE} --enable=warning,style,performance --std=c++20
                --suppress=missingInclude
                --suppressions-list=${CMAKE_SOURCE_DIR}/.cppcheck-suppressions
                -I ${CMAKE_SOURCE_DIR}/include
                ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/tools
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "cppcheck over src/")
endif()

# Opt-in (default off, zero effect on a normal build): the engine parses external glTF/JSON/EXR files, where memory-safety tooling earns its keep. `cmake -B build -DENGINE_SANITIZE=ON`.
option(ENGINE_SANITIZE "Build with AddressSanitizer + UndefinedBehaviorSanitizer" OFF)
if(ENGINE_SANITIZE)
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
# Built as a separate OBJECT library rather than a per-source override: CXX_CLANG_TIDY is target-level-only in CMake, so keeping clang-tidy off vendored sources requires a target that never sets the property.
add_library(vendored_no_tidy OBJECT ${IMGUI_SOURCES} src/scene/cgltf_impl.cpp)
target_include_directories(vendored_no_tidy PRIVATE ${IMGUI_DIR} ${IMGUI_DIR}/backends ${CGLTF_DIR})
target_link_libraries(vendored_no_tidy PRIVATE glfw)
target_compile_options(vendored_no_tidy PRIVATE -w)

add_executable(engine
    src/main.cpp
    src/platform/window.cpp
    src/gfx/gl_debug.cpp
    src/gfx/shader_program.cpp
    src/gfx/texture.cpp
    src/gfx/hdr_image.cpp
    src/gfx/post_process_pass.cpp
    src/gfx/ocio_display_transform.cpp
    src/scene/bsdf.cpp
    src/scene/camera.cpp
    src/scene/debug_camera_controller.cpp
    src/scene/embree_accel.cpp
    src/scene/environment_map.cpp
    src/scene/gbuffer_shading.cpp
    src/scene/gltf_loader.cpp
    src/scene/light.cpp
    src/scene/material.cpp
    src/scene/material_binding.cpp
    src/scene/path_tracer.cpp
    src/scene/path_trace_driver.cpp
    src/scene/rasterizer.cpp
    src/scene/ray_types.cpp
    src/scene/thread_pool.cpp
    src/scene/sampler.cpp
    src/scene/shading_scene.cpp
    src/config/profile_config.cpp
    src/config/scene_config.cpp
    src/debug/system_info.cpp
    src/debug/host_info.cpp
    src/debug/frame_stats.cpp
    src/debug/gpu_timer.cpp
    src/debug/memory_tracker.cpp
    src/debug/histogram.cpp
    src/debug/hud_overlay.cpp
    src/debug/spec_report.cpp
    src/debug/perf_dashboard.cpp
    $<TARGET_OBJECTS:vendored_no_tidy>)
target_include_directories(engine PRIVATE include ${IMGUI_DIR} ${IMGUI_DIR}/backends ${CGLTF_DIR} ${JSON_DIR}
                           ${ENGINE_GENERATED_DIR})
add_dependencies(engine engine_git_sha)
target_compile_definitions(engine PRIVATE
    GL_SILENCE_DEPRECATION
    ASSET_ROOT_DIR="${CMAKE_SOURCE_DIR}/assets"
    ${ENGINE_PROVENANCE_DEFINITIONS})
target_compile_options(engine PRIVATE -Wall -Wextra -Werror -march=native)
target_link_libraries(engine PRIVATE
    glfw GLEW::GLEW OpenGL::GL glm::glm
    OpenEXR::OpenEXR Imath::Imath OpenColorIO::OpenColorIO embree)
if(CLANG_TIDY_EXE)
    set_target_properties(engine PROPERTIES CXX_CLANG_TIDY "${CLANG_TIDY_COMMAND}")
endif()
# Free codegen speedup for the path tracer's hot loop (ray traversal, BSDF eval), no behavior/quality change. No-op if the toolchain lacks IPO support, mirroring the clang-tidy/cppcheck no-op-if-absent pattern above.
if(IPO_SUPPORTED)
    set_target_properties(engine PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

add_executable(test_pattern tools/test_pattern.cpp)
target_compile_definitions(test_pattern PRIVATE
    ASSET_ROOT_DIR="${CMAKE_SOURCE_DIR}/assets")
target_compile_options(test_pattern PRIVATE -Wall -Wextra -Werror)
target_link_libraries(test_pattern PRIVATE OpenEXR::OpenEXR Imath::Imath)

add_executable(downsample tools/downsample.cpp)
target_compile_options(downsample PRIVATE -Wall -Wextra -Werror)
target_link_libraries(downsample PRIVATE OpenEXR::OpenEXR Imath::Imath)

# One-off asset-prep for Houdini glTF exports: synthesizes a TANGENT accessor for any primitive
# that lacks one, in place. Every other attribute (POSITION/NORMAL/TEXCOORD_0/COLOR_0/...), every
# material, and every other primitive is patched, not rebuilt -- gltf_loader.cpp's own default-texture
# substitution (and its acceptance of a primitive with no material at all) handles the rest.
add_executable(gltf_tangent tools/gltf_tangent.cpp)
target_include_directories(gltf_tangent PRIVATE ${JSON_DIR})
target_compile_options(gltf_tangent PRIVATE -Wall -Wextra -Werror)
target_link_libraries(gltf_tangent PRIVATE glm::glm)

# Offline generator for src/scene/blue_noise_mask.inc, the void-and-cluster dither array sampler.cpp bakes in. Grouped
# with gltf_tangent rather than with the validate tools below: it produces a committed artifact, it does not check one,
# so it stays out of the ctest loop. -march=native/IPO because it is convolution-bound and only ever run by hand.
add_executable(bluenoise_mask tools/bluenoise_mask.cpp src/gfx/hdr_image.cpp)
target_include_directories(bluenoise_mask PRIVATE include)
target_compile_options(bluenoise_mask PRIVATE -Wall -Wextra -Werror -march=native)
target_link_libraries(bluenoise_mask PRIVATE glm::glm OpenEXR::OpenEXR Imath::Imath)
if(IPO_SUPPORTED)
    set_target_properties(bluenoise_mask PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# Offline generator for src/scene/albedo_table.inc, the Kulla-Conty energy tables bsdf.cpp bakes in. Grouped with
# bluenoise_mask for the same reason: it produces a committed artifact, it does not check one, so it stays out of the
# ctest loop. Neither -march=native nor IPO, unlike every other tool here -- the output is committed source and must
# reproduce bit-for-bit on any machine, and FMA contraction is free to differ; the bake runs in seconds regardless.
add_executable(albedo_table tools/albedo_table.cpp)
target_include_directories(albedo_table PRIVATE include)
target_compile_options(albedo_table PRIVATE -Wall -Wextra -Werror)
target_link_libraries(albedo_table PRIVATE glm::glm)

# Headless correctness checks (Phase 4): no test framework exists yet, so these follow test_pattern/downsample's standalone-CLI convention (non-zero exit on failure).
# -march=native/IPO mirror the engine target's flags so these tools validate the exact codegen that ships, not a differently-optimized build of the same math.
# Shared validator harness (tools/check.h, tools/stats.h). Deliberately depends on nothing but glm: the light
# validators (sampler, embree) must not be forced to link Embree or OpenEXR just to reach a chi-square survival
# function. Built with the same flags as the tools it serves so the harness is not a different codegen from them.
add_library(engine_check OBJECT tools/check.cpp)
target_include_directories(engine_check PUBLIC tools)
target_compile_options(engine_check PRIVATE -Wall -Wextra -Werror -march=native)
target_link_libraries(engine_check PRIVATE glm::glm)

# Guards the harness's own invariants -- NaN rejection and name-derived seeds -- which nothing else would notice
# regressing, because a broken assert macro reports success.
add_executable(check_selftest tools/check_selftest.cpp $<TARGET_OBJECTS:engine_check>)
target_include_directories(check_selftest PRIVATE tools)
target_compile_options(check_selftest PRIVATE -Wall -Wextra -Werror -march=native)
target_link_libraries(check_selftest PRIVATE glm::glm)

add_executable(embree_validate tools/embree_validate.cpp src/scene/embree_accel.cpp src/scene/ray_types.cpp
               $<TARGET_OBJECTS:engine_check>)
target_include_directories(embree_validate PRIVATE include tools)
target_compile_options(embree_validate PRIVATE -Wall -Wextra -Werror -march=native)
target_link_libraries(embree_validate PRIVATE glm::glm embree)
if(IPO_SUPPORTED)
    set_target_properties(embree_validate PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

add_executable(bsdf_validate tools/bsdf_validate.cpp src/scene/bsdf.cpp src/scene/sampler.cpp
               $<TARGET_OBJECTS:engine_check>)
target_include_directories(bsdf_validate PRIVATE include tools)
target_compile_options(bsdf_validate PRIVATE -Wall -Wextra -Werror -march=native)
target_link_libraries(bsdf_validate PRIVATE glm::glm)
if(IPO_SUPPORTED)
    set_target_properties(bsdf_validate PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

add_executable(sampler_validate tools/sampler_validate.cpp src/scene/sampler.cpp src/debug/power_spectrum.cpp
               $<TARGET_OBJECTS:engine_check>)
target_include_directories(sampler_validate PRIVATE include tools)
target_compile_options(sampler_validate PRIVATE -Wall -Wextra -Werror -march=native)
target_link_libraries(sampler_validate PRIVATE glm::glm)
if(IPO_SUPPORTED)
    set_target_properties(sampler_validate PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

add_executable(nee_validate tools/nee_validate.cpp src/scene/bsdf.cpp src/scene/sampler.cpp
               src/scene/environment_map.cpp src/scene/light.cpp src/gfx/hdr_image.cpp
               $<TARGET_OBJECTS:engine_check>)
target_include_directories(nee_validate PRIVATE include tools)
target_compile_options(nee_validate PRIVATE -Wall -Wextra -Werror -march=native)
target_link_libraries(nee_validate PRIVATE glm::glm OpenEXR::OpenEXR Imath::Imath)
if(IPO_SUPPORTED)
    set_target_properties(nee_validate PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

add_executable(rasterizer_validate tools/rasterizer_validate.cpp src/scene/rasterizer.cpp
               src/scene/gbuffer_shading.cpp src/scene/embree_accel.cpp src/scene/ray_types.cpp
               src/scene/shading_scene.cpp src/scene/bsdf.cpp src/scene/sampler.cpp
               src/scene/camera.cpp src/scene/thread_pool.cpp src/gfx/hdr_image.cpp
               $<TARGET_OBJECTS:engine_check>)
target_include_directories(rasterizer_validate PRIVATE include tools)
target_compile_options(rasterizer_validate PRIVATE -Wall -Wextra -Werror -march=native)
target_link_libraries(rasterizer_validate PRIVATE glm::glm embree OpenEXR::OpenEXR Imath::Imath)
if(IPO_SUPPORTED)
    set_target_properties(rasterizer_validate PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

add_executable(integrator_validate tools/integrator_validate.cpp src/scene/path_tracer.cpp
               src/scene/gbuffer_shading.cpp src/scene/embree_accel.cpp src/scene/ray_types.cpp
               src/scene/shading_scene.cpp src/scene/bsdf.cpp src/scene/sampler.cpp
               src/scene/environment_map.cpp src/scene/light.cpp src/scene/camera.cpp src/scene/thread_pool.cpp
               src/scene/material_binding.cpp src/config/scene_config.cpp src/gfx/hdr_image.cpp
               $<TARGET_OBJECTS:engine_check>)
# material_binding/scene_config and the asset root are for checkMaterialBinding, which resolves real assets/materials/*.json files against named instances -- the materialOverrides path no other validator touches.
target_include_directories(integrator_validate PRIVATE include tools ${JSON_DIR})
target_compile_definitions(integrator_validate PRIVATE
    ASSET_ROOT_DIR="${CMAKE_SOURCE_DIR}/assets")
target_compile_options(integrator_validate PRIVATE -Wall -Wextra -Werror -march=native)
target_link_libraries(integrator_validate PRIVATE glm::glm embree OpenEXR::OpenEXR Imath::Imath)
if(IPO_SUPPORTED)
    set_target_properties(integrator_validate PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# Progressive-accumulation driver (src/scene/path_trace_driver.cpp), previously the largest CPU component with no
# coverage at all. Links the same transport sources integrator_validate does, plus the driver itself.
add_executable(driver_validate tools/driver_validate.cpp src/scene/path_trace_driver.cpp
               src/scene/path_tracer.cpp src/scene/gbuffer_shading.cpp src/scene/embree_accel.cpp
               src/scene/ray_types.cpp src/scene/shading_scene.cpp src/scene/bsdf.cpp src/scene/sampler.cpp
               src/scene/environment_map.cpp src/scene/light.cpp src/scene/camera.cpp
               src/scene/thread_pool.cpp src/gfx/hdr_image.cpp
               $<TARGET_OBJECTS:engine_check>)
target_include_directories(driver_validate PRIVATE include tools)
target_compile_options(driver_validate PRIVATE -Wall -Wextra -Werror -march=native)
target_link_libraries(driver_validate PRIVATE glm::glm embree OpenEXR::OpenEXR Imath::Imath)
if(IPO_SUPPORTED)
    set_target_properties(driver_validate PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# File-boundary validation (gfx/hdr_image.cpp EXR round trip, config/*.cpp JSON parsing). The EXR path was linked into
# three validators and invoked by none of them; profile_config.cpp had no coverage at all.
add_executable(io_validate tools/io_validate.cpp src/gfx/hdr_image.cpp src/config/scene_config.cpp
               src/config/profile_config.cpp src/scene/camera.cpp src/debug/bench_log.cpp src/debug/host_info.cpp
               src/debug/memory_tracker.cpp
               $<TARGET_OBJECTS:engine_check>)
target_include_directories(io_validate PRIVATE include tools ${JSON_DIR} ${ENGINE_GENERATED_DIR})
add_dependencies(io_validate engine_git_sha)
target_compile_definitions(io_validate PRIVATE ASSET_ROOT_DIR="${CMAKE_SOURCE_DIR}/assets"
                           ${ENGINE_PROVENANCE_DEFINITIONS})
target_compile_options(io_validate PRIVATE -Wall -Wextra -Werror -march=native)
target_link_libraries(io_validate PRIVATE glm::glm OpenEXR::OpenEXR Imath::Imath ZLIB::ZLIB embree)
if(IPO_SUPPORTED)
    set_target_properties(io_validate PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# CPU-evaluable display and statistics path. The OCIO transform's pinned-config property was asserted only by a
# comment in its own header; frame_stats is testable exactly now that tick() takes an injectable clock.
add_executable(display_validate tools/display_validate.cpp src/gfx/ocio_cpu_transform.cpp
               src/debug/frame_stats.cpp
               $<TARGET_OBJECTS:engine_check>)
target_include_directories(display_validate PRIVATE include tools)
target_compile_options(display_validate PRIVATE -Wall -Wextra -Werror -march=native)
target_link_libraries(display_validate PRIVATE glm::glm OpenColorIO::OpenColorIO)
if(IPO_SUPPORTED)
    set_target_properties(display_validate PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# Headless before/after comparison renders (tools/render_beauty.cpp). Not a correctness gate and deliberately absent from the add_test loop below: its output is a picture for a human to look at, and a full-scene render is far slower than any validator. Needs OCIO (the display transform, evaluated on the CPU) and zlib (PNG encode) on top of the usual render dependencies.
add_executable(render_beauty tools/render_beauty.cpp src/debug/power_spectrum.cpp src/scene/path_tracer.cpp
               src/scene/gbuffer_shading.cpp src/scene/embree_accel.cpp src/scene/ray_types.cpp
               src/scene/shading_scene.cpp src/scene/bsdf.cpp src/scene/sampler.cpp
               src/scene/environment_map.cpp src/scene/light.cpp src/scene/camera.cpp src/scene/thread_pool.cpp
               src/scene/gltf_loader.cpp src/scene/material.cpp src/scene/material_binding.cpp
               src/config/scene_config.cpp src/config/profile_config.cpp src/gfx/hdr_image.cpp
               src/gfx/ocio_cpu_transform.cpp src/debug/bench_log.cpp src/debug/host_info.cpp
               src/debug/memory_tracker.cpp
               $<TARGET_OBJECTS:vendored_no_tidy> $<TARGET_OBJECTS:engine_check>)
target_include_directories(render_beauty PRIVATE include tools ${CGLTF_DIR} ${JSON_DIR} ${ENGINE_GENERATED_DIR})
add_dependencies(render_beauty engine_git_sha)
target_compile_definitions(render_beauty PRIVATE ASSET_ROOT_DIR="${CMAKE_SOURCE_DIR}/assets"
                           ${ENGINE_PROVENANCE_DEFINITIONS})
target_compile_options(render_beauty PRIVATE -Wall -Wextra -Werror -march=native)
target_link_libraries(render_beauty PRIVATE glm::glm embree OpenEXR::OpenEXR Imath::Imath
                       OpenColorIO::OpenColorIO ZLIB::ZLIB glfw)
if(IPO_SUPPORTED)
    set_target_properties(render_beauty PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# Timing harness, not a correctness gate: deliberately absent from the add_test loop below (a benchmark's pass/fail is a human reading a number, and it would make ctest as slow as its worst --triangles sweep). Same sources and -march=native/IPO flags as rasterizer_validate, so it times the exact codegen that ships.
add_executable(raster_bench tools/raster_bench.cpp src/scene/rasterizer.cpp
               src/scene/gbuffer_shading.cpp src/scene/embree_accel.cpp src/scene/ray_types.cpp
               src/scene/shading_scene.cpp src/scene/bsdf.cpp src/scene/sampler.cpp
               src/scene/camera.cpp src/scene/thread_pool.cpp src/gfx/hdr_image.cpp
               src/debug/bench_log.cpp src/debug/host_info.cpp src/debug/memory_tracker.cpp)
target_include_directories(raster_bench PRIVATE include ${JSON_DIR} ${ENGINE_GENERATED_DIR})
add_dependencies(raster_bench engine_git_sha)
target_compile_definitions(raster_bench PRIVATE ${ENGINE_PROVENANCE_DEFINITIONS})
target_compile_options(raster_bench PRIVATE -Wall -Wextra -Werror -march=native)
target_link_libraries(raster_bench PRIVATE glm::glm embree OpenEXR::OpenEXR Imath::Imath ZLIB::ZLIB)
if(IPO_SUPPORTED)
    set_target_properties(raster_bench PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# All correctness tools run under `ctest --test-dir build`; previously buildable but unrun, so a regression only surfaced if someone remembered to invoke them by hand.
enable_testing()

# Concurrent checks would each build a pool sized to hardware_concurrency(), so `ctest -j` would oversubscribe by that
# factor. Each discovered test declares PROCESSORS to match, so ctest's job pool accounts for what it will actually use.
include(ProcessorCount)
ProcessorCount(ENGINE_HOST_PROCESSORS)
if(ENGINE_HOST_PROCESSORS EQUAL 0)
    set(ENGINE_HOST_PROCESSORS 1)
endif()
math(EXPR ENGINE_TEST_THREADS "${ENGINE_HOST_PROCESSORS} / 4")
if(ENGINE_TEST_THREADS LESS 1)
    set(ENGINE_TEST_THREADS 1)
endif()

include(${CMAKE_SOURCE_DIR}/cmake/EngineChecks.cmake)

# Image-level gates. Not part of a validator binary because they need the real scene, the real glTF and the real
# material stack -- what they assert is a property of the shipping pipeline, not of a synthetic fixture.
# Deliberately small and short: both are statements about the ESTIMATOR, which does not need a converged image to
# test, and a gate that takes minutes stops being run.
add_test(NAME image.determinism
         COMMAND render_beauty --scene scenes/cornell.json --width 48 --height 36 --passes 8
                 --assert-deterministic)
set_tests_properties(image.determinism PROPERTIES LABELS "image;fast;exact" TIMEOUT 300)

add_test(NAME image.convergence
         COMMAND render_beauty --scene scenes/cornell.json --width 48 --height 36 --passes 64
                 --assert-converged)
set_tests_properties(image.convergence PROPERTIES LABELS "image;slow;statistical" TIMEOUT 900
                     PROCESSORS ${ENGINE_TEST_THREADS})

# Migrated to tools/check.h: one ctest entry per check, discovered from the binary itself.
engine_discover_checks(check_selftest)
engine_discover_checks(sampler_validate)
engine_discover_checks(embree_validate)
engine_discover_checks(rasterizer_validate)
engine_discover_checks(nee_validate)
engine_discover_checks(integrator_validate)
engine_discover_checks(bsdf_validate)
engine_discover_checks(driver_validate)
engine_discover_checks(io_validate)
engine_discover_checks(display_validate)

