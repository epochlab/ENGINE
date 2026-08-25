cmake_minimum_required(VERSION 3.21)
project(Engine LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Single-config generators (Makefiles/Ninja) default to an unoptimized build if CMAKE_BUILD_TYPE is never set -- silently ~40x slower for CPU-bound work (measured: a 5M-triangle glTF load went from ~1.1s Release to 47s+ and still running, unoptimized). Multi-config generators (Xcode) ignore this and use CMAKE_CONFIGURATION_TYPES instead, hence the guard.
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
    message(FATAL_ERROR "third_party/imgui is empty — run: git submodule update --init")
endif()

set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp)

set(CGLTF_DIR ${CMAKE_SOURCE_DIR}/third_party/cgltf)

# Vendored code: not ours to fix (imgui_impl_glfw.cpp pulls in Apple's ApplicationServices.h via GLFW's native-handle header, which warns against itself; cgltf_impl.cpp is C99 with implicit conversions -Wall/-Wextra flags). Built as a separate OBJECT library, not just a per-source COMPILE_OPTIONS/CXX_CLANG_TIDY override on the engine target: CXX_CLANG_TIDY is a target-level-only property in CMake (no working per-source override), so the only way to keep clang-tidy off vendored sources is to put them on a target that never has the property set at all.
add_library(vendored_no_tidy OBJECT ${IMGUI_SOURCES} src/scene/cgltf_impl.cpp)
target_include_directories(vendored_no_tidy PRIVATE ${IMGUI_DIR} ${IMGUI_DIR}/backends ${CGLTF_DIR})
target_compile_options(vendored_no_tidy PRIVATE -w)

add_executable(engine
    src/main.cpp
    src/platform/window.cpp
    src/gfx/gl_debug.cpp
    src/gfx/shader_program.cpp
    src/gfx/mesh.cpp
    src/gfx/texture.cpp
    src/gfx/hdr_image.cpp
    src/gfx/cubemap_texture.cpp
    src/gfx/env_prefilter_pass.cpp
    src/gfx/hdr_framebuffer.cpp
    src/gfx/post_process_pass.cpp
    src/gfx/ocio_display_transform.cpp
    src/scene/bvh.cpp
    src/scene/camera.cpp
    src/scene/debug_camera_controller.cpp
    src/scene/environment_map.cpp
    src/scene/frustum.cpp
    src/scene/gltf_loader.cpp
    src/scene/sh_irradiance.cpp
    src/scene/shading_scene.cpp
    src/config/json_scan.cpp
    src/config/profile_config.cpp
    src/config/scene_config.cpp
    src/debug/system_info.cpp
    src/debug/frame_stats.cpp
    src/debug/gpu_timer.cpp
    src/debug/memory_tracker.cpp
    src/debug/histogram.cpp
    src/debug/hud_overlay.cpp
    $<TARGET_OBJECTS:vendored_no_tidy>)
target_include_directories(engine PRIVATE include ${IMGUI_DIR} ${IMGUI_DIR}/backends ${CGLTF_DIR})
target_compile_definitions(engine PRIVATE
    GL_SILENCE_DEPRECATION
    ASSET_ROOT_DIR="${CMAKE_SOURCE_DIR}/assets")
target_compile_options(engine PRIVATE -Wall -Wextra -Werror)
target_link_libraries(engine PRIVATE
    glfw GLEW::GLEW OpenGL::GL glm::glm
    OpenEXR::OpenEXR Imath::Imath OpenColorIO::OpenColorIO)
if(CLANG_TIDY_EXE)
    set_target_properties(engine PROPERTIES CXX_CLANG_TIDY "${CLANG_TIDY_COMMAND}")
endif()

add_executable(test_pattern tools/test_pattern.cpp)
target_compile_definitions(test_pattern PRIVATE
    ASSET_ROOT_DIR="${CMAKE_SOURCE_DIR}/assets")
target_compile_options(test_pattern PRIVATE -Wall -Wextra -Werror)
target_link_libraries(test_pattern PRIVATE OpenEXR::OpenEXR Imath::Imath)

add_executable(downsample tools/downsample.cpp)
target_compile_options(downsample PRIVATE -Wall -Wextra -Werror)
target_link_libraries(downsample PRIVATE OpenEXR::OpenEXR Imath::Imath)

# Headless correctness checks (Phase 4) -- no test framework exists in this repo yet, so these follow test_pattern/downsample's standalone-CLI convention: non-zero exit on failure.
add_executable(bvh_validate tools/bvh_validate.cpp src/scene/bvh.cpp)
target_include_directories(bvh_validate PRIVATE include)
target_compile_options(bvh_validate PRIVATE -Wall -Wextra -Werror)
target_link_libraries(bvh_validate PRIVATE glm::glm)

add_executable(furnace_test tools/furnace_test.cpp src/scene/sh_irradiance.cpp)
target_include_directories(furnace_test PRIVATE include)
target_compile_options(furnace_test PRIVATE -Wall -Wextra -Werror)
target_link_libraries(furnace_test PRIVATE glm::glm)
