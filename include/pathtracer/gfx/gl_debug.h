#pragma once

namespace pathtracer::gfx {

// Drains the GL error queue, logging any non-GL_NO_ERROR codes to stderr tagged with the call site. Called by
// GL_CALL below, and safe to call directly.
void checkError(const char* file, int line);

// True if the current context exposes GL_KHR_debug. Queried, never assumed: macOS's GL 4.1 core driver is expected
// to lack it, but that is not guaranteed.
bool khrDebugAvailable();

}  // namespace pathtracer::gfx

// Wraps a GL call with an error check in debug builds only, avoiding per-frame glGetError() in Release. NDEBUG is
// CMake's own Debug/Release signal.
#ifndef NDEBUG
#define GL_CALL(x)                                    \
    do {                                               \
        x;                                             \
        ::pathtracer::gfx::checkError(__FILE__, __LINE__); \
    } while (false)
#else
#define GL_CALL(x) x
#endif
