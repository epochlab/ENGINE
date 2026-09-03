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

find_program(CPPCHECK_EXE NAMES cppcheck)
if(CPPCHECK_EXE)
    add_custom_target(cppcheck
        COMMAND ${CPPCHECK_EXE} --enable=warning,style,performance --std=c++20
                --suppress=missingInclude -I ${CMAKE_SOURCE_DIR}/include
                ${CMAKE_SOURCE_DIR}/src
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
    src/debug/frame_stats.cpp
    src/debug/gpu_timer.cpp
    src/debug/memory_tracker.cpp
    src/debug/histogram.cpp
    src/debug/hud_overlay.cpp
    $<TARGET_OBJECTS:vendored_no_tidy>)
target_include_directories(engine PRIVATE include ${IMGUI_DIR} ${IMGUI_DIR}/backends ${CGLTF_DIR} ${JSON_DIR})
target_compile_definitions(engine PRIVATE
    GL_SILENCE_DEPRECATION
    ASSET_ROOT_DIR="${CMAKE_SOURCE_DIR}/assets")
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

# One-off asset-prep for bare Houdini POSITION/NORMAL/TEXCOORD_0 glTF exports: synthesizes a
# placeholder TANGENT and a texture-less material in place, so gltf_loader.cpp's default-texture
# substitution takes over and scene_config's global material scalars (see
# assets/scenes/cornell.json) drive the look instead.
add_executable(gltf_prep tools/gltf_prep.cpp)
target_include_directories(gltf_prep PRIVATE ${JSON_DIR})
target_compile_options(gltf_prep PRIVATE -Wall -Wextra -Werror)
target_link_libraries(gltf_prep PRIVATE glm::glm)

# Headless correctness checks (Phase 4): no test framework exists yet, so these follow test_pattern/downsample's standalone-CLI convention (non-zero exit on failure).
# -march=native/IPO mirror the engine target's flags so these tools validate the exact codegen that ships, not a differently-optimized build of the same math.
add_executable(embree_validate tools/embree_validate.cpp src/scene/embree_accel.cpp src/scene/ray_types.cpp)
target_include_directories(embree_validate PRIVATE include)
target_compile_options(embree_validate PRIVATE -Wall -Wextra -Werror -march=native)
target_link_libraries(embree_validate PRIVATE glm::glm embree)
if(IPO_SUPPORTED)
    set_target_properties(embree_validate PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

add_executable(bsdf_validate tools/bsdf_validate.cpp src/scene/bsdf.cpp src/scene/sampler.cpp)
target_include_directories(bsdf_validate PRIVATE include)
target_compile_options(bsdf_validate PRIVATE -Wall -Wextra -Werror -march=native)
target_link_libraries(bsdf_validate PRIVATE glm::glm)
if(IPO_SUPPORTED)
    set_target_properties(bsdf_validate PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

add_executable(nee_validate tools/nee_validate.cpp src/scene/bsdf.cpp src/scene/sampler.cpp
               src/scene/environment_map.cpp src/gfx/hdr_image.cpp)
target_include_directories(nee_validate PRIVATE include)
target_compile_options(nee_validate PRIVATE -Wall -Wextra -Werror -march=native)
target_link_libraries(nee_validate PRIVATE glm::glm OpenEXR::OpenEXR Imath::Imath)
if(IPO_SUPPORTED)
    set_target_properties(nee_validate PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

add_executable(rasterizer_validate tools/rasterizer_validate.cpp src/scene/rasterizer.cpp
               src/scene/gbuffer_shading.cpp src/scene/embree_accel.cpp src/scene/ray_types.cpp
               src/scene/shading_scene.cpp src/scene/bsdf.cpp src/scene/sampler.cpp
               src/scene/camera.cpp src/scene/thread_pool.cpp src/gfx/hdr_image.cpp)
target_include_directories(rasterizer_validate PRIVATE include)
target_compile_options(rasterizer_validate PRIVATE -Wall -Wextra -Werror -march=native)
target_link_libraries(rasterizer_validate PRIVATE glm::glm embree OpenEXR::OpenEXR Imath::Imath)
if(IPO_SUPPORTED)
    set_target_properties(rasterizer_validate PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

add_executable(integrator_validate tools/integrator_validate.cpp src/scene/path_tracer.cpp
               src/scene/gbuffer_shading.cpp src/scene/embree_accel.cpp src/scene/ray_types.cpp
               src/scene/shading_scene.cpp src/scene/bsdf.cpp src/scene/sampler.cpp
               src/scene/environment_map.cpp src/scene/camera.cpp src/scene/thread_pool.cpp
               src/gfx/hdr_image.cpp)
target_include_directories(integrator_validate PRIVATE include)
target_compile_options(integrator_validate PRIVATE -Wall -Wextra -Werror -march=native)
target_link_libraries(integrator_validate PRIVATE glm::glm embree OpenEXR::OpenEXR Imath::Imath)
if(IPO_SUPPORTED)
    set_target_properties(integrator_validate PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# Timing harness, not a correctness gate: deliberately absent from the add_test loop below (a benchmark's pass/fail is a human reading a number, and it would make ctest as slow as its worst --triangles sweep). Same sources and -march=native/IPO flags as rasterizer_validate, so it times the exact codegen that ships.
add_executable(raster_bench tools/raster_bench.cpp src/scene/rasterizer.cpp
               src/scene/gbuffer_shading.cpp src/scene/embree_accel.cpp src/scene/ray_types.cpp
               src/scene/shading_scene.cpp src/scene/bsdf.cpp src/scene/sampler.cpp
               src/scene/camera.cpp src/scene/thread_pool.cpp src/gfx/hdr_image.cpp)
target_include_directories(raster_bench PRIVATE include)
target_compile_options(raster_bench PRIVATE -Wall -Wextra -Werror -march=native)
target_link_libraries(raster_bench PRIVATE glm::glm embree OpenEXR::OpenEXR Imath::Imath)
if(IPO_SUPPORTED)
    set_target_properties(raster_bench PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# All five correctness tools run under `ctest --test-dir build`; previously buildable but unrun, so a regression only surfaced if someone remembered to invoke them by hand.
enable_testing()
foreach(validator embree_validate bsdf_validate nee_validate rasterizer_validate integrator_validate)
    add_test(NAME ${validator} COMMAND ${validator})
endforeach()
