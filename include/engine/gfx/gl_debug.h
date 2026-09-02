#pragma once

namespace engine::gfx {

// Drains the GL error queue, logging any real (non-GL_NO_ERROR) codes to stderr, tagged with the call site. Called by GL_CALL below; safe to call directly too.
void checkError(const char* file, int line);

// True if the current context exposes GL_KHR_debug. Queried at runtime, never assumed: macOS's GL 4.1 core driver is expected (not guaranteed) to lack it.
bool khrDebugAvailable();

}  // namespace engine::gfx

// Wraps a GL call with an error check in debug builds only, avoiding per-frame glGetError() overhead in Release. NDEBUG is CMake's own Debug/Release signal (Release defines it, Debug doesn't); no project-specific macro needed.
#ifndef NDEBUG
#define GL_CALL(x)                                    \
    do {                                               \
        x;                                             \
        ::engine::gfx::checkError(__FILE__, __LINE__); \
    } while (false)
#else
#define GL_CALL(x) x
#endif
