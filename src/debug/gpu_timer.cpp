#include "engine/debug/gpu_timer.h"

#include <GL/glew.h>

namespace engine::debug {

bool gpuTimerQueryAvailable() {
    return GLEW_ARB_timer_query != 0;
}

GpuTimer::GpuTimer() {
    glGenQueries(2, queries_);
}

GpuTimer::~GpuTimer() {
    if (queries_[0] != 0) {
        glDeleteQueries(2, queries_);
    }
}

GpuTimer::GpuTimer(GpuTimer&& other) noexcept
    : writeIndex_(other.writeIndex_), lastMs_(other.lastMs_) {
    queries_[0] = other.queries_[0];
    queries_[1] = other.queries_[1];
    slotUsed_[0] = other.slotUsed_[0];
    slotUsed_[1] = other.slotUsed_[1];
    other.queries_[0] = 0;
    other.queries_[1] = 0;
}

GpuTimer& GpuTimer::operator=(GpuTimer&& other) noexcept {
    if (this != &other) {
        if (queries_[0] != 0) {
            glDeleteQueries(2, queries_);
        }
        queries_[0] = other.queries_[0];
        queries_[1] = other.queries_[1];
        writeIndex_ = other.writeIndex_;
        slotUsed_[0] = other.slotUsed_[0];
        slotUsed_[1] = other.slotUsed_[1];
        lastMs_ = other.lastMs_;
        other.queries_[0] = 0;
        other.queries_[1] = 0;
    }
    return *this;
}

void GpuTimer::begin() {
    glBeginQuery(GL_TIME_ELAPSED, queries_[static_cast<unsigned>(writeIndex_)]);
}

void GpuTimer::end() {
    glEndQuery(GL_TIME_ELAPSED);
    slotUsed_[static_cast<unsigned>(writeIndex_)] = true;
}

float GpuTimer::millisecondsElapsed() {
    const int readIndex = 1 - writeIndex_;
    if (slotUsed_[static_cast<unsigned>(readIndex)]) {
        GLint available = 0;
        glGetQueryObjectiv(queries_[static_cast<unsigned>(readIndex)], GL_QUERY_RESULT_AVAILABLE,
                            &available);
        if (available != 0) {
            GLuint64 ns = 0;
            glGetQueryObjectui64v(queries_[static_cast<unsigned>(readIndex)], GL_QUERY_RESULT,
                                   &ns);
            lastMs_ = static_cast<float>(ns) / 1.0e6F;
        }
    }
    writeIndex_ = readIndex;
    return lastMs_;
}

}  // namespace engine::debug
