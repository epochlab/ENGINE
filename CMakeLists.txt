cmake_minimum_required(VERSION 3.21)
project(Engine LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenGL REQUIRED)
find_package(glfw3 3.5 REQUIRED)
find_package(GLEW REQUIRED)
find_package(glm REQUIRED)
find_package(Imath REQUIRED)
find_package(OpenEXR REQUIRED)
find_package(OpenColorIO REQUIRED)

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
# Vendored code: not ours to fix (imgui_impl_glfw.cpp pulls in Apple's
# ApplicationServices.h via GLFW's native-handle header, which warns against itself).
set_source_files_properties(${IMGUI_SOURCES} PROPERTIES COMPILE_OPTIONS "-w")

set(CGLTF_DIR ${CMAKE_SOURCE_DIR}/third_party/cgltf)
# Vendored code: not ours to fix (C99, implicit conversions -Wall/-Wextra flag).
set_source_files_properties(src/scene/cgltf_impl.cpp PROPERTIES COMPILE_OPTIONS "-w")

add_executable(engine
    src/main.cpp
    src/platform/window.cpp
    src/gfx/gl_debug.cpp
    src/gfx/shader_program.cpp
    src/gfx/mesh.cpp
    src/gfx/texture.cpp
    src/gfx/hdr_framebuffer.cpp
    src/gfx/post_process_pass.cpp
    src/gfx/ocio_display_transform.cpp
    src/scene/camera.cpp
    src/scene/cgltf_impl.cpp
    src/scene/frustum.cpp
    src/scene/gltf_loader.cpp
    src/debug/system_info.cpp
    src/debug/frame_stats.cpp
    src/debug/gpu_timer.cpp
    src/debug/memory_tracker.cpp
    src/debug/hud_overlay.cpp
    ${IMGUI_SOURCES})
target_include_directories(engine PRIVATE include ${IMGUI_DIR} ${IMGUI_DIR}/backends ${CGLTF_DIR})
target_compile_definitions(engine PRIVATE
    GL_SILENCE_DEPRECATION
    ASSET_ROOT_DIR="${CMAKE_SOURCE_DIR}/assets")
target_compile_options(engine PRIVATE -Wall -Wextra -Werror)
target_link_libraries(engine PRIVATE
    glfw GLEW::GLEW OpenGL::GL glm::glm
    OpenEXR::OpenEXR Imath::Imath OpenColorIO::OpenColorIO)

add_executable(test_pattern tools/test_pattern.cpp)
target_compile_definitions(test_pattern PRIVATE
    ASSET_ROOT_DIR="${CMAKE_SOURCE_DIR}/assets")
target_compile_options(test_pattern PRIVATE -Wall -Wextra -Werror)
target_link_libraries(test_pattern PRIVATE OpenEXR::OpenEXR Imath::Imath)

add_executable(downsample tools/downsample.cpp)
target_compile_options(downsample PRIVATE -Wall -Wextra -Werror)
target_link_libraries(downsample PRIVATE OpenEXR::OpenEXR Imath::Imath)
