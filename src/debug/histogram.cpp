#include "engine/debug/histogram.h"

#include <utility>

#include <GL/glew.h>

#include "engine/debug/memory_tracker.h"
#include "engine/gfx/gl_debug.h"

namespace engine::debug {

Histogram::Histogram() {
    GL_CALL(glGenFramebuffers(1, &downsampleFbo_));
    GL_CALL(glGenTextures(1, &downsampleTexture_));
    GL_CALL(glBindTexture(GL_TEXTURE_2D, downsampleTexture_));
    GL_CALL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, kWidth, kHeight, 0, GL_RGB, GL_UNSIGNED_BYTE,
                          nullptr));
    GL_CALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
    GL_CALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    GL_CALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    GL_CALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
    GL_CALL(glBindFramebuffer(GL_FRAMEBUFFER, downsampleFbo_));
    GL_CALL(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                                    downsampleTexture_, 0));
    GL_CALL(glBindFramebuffer(GL_FRAMEBUFFER, 0));
    GL_CALL(glBindTexture(GL_TEXTURE_2D, 0));

    const auto pboBytes = static_cast<std::size_t>(kWidth) * static_cast<std::size_t>(kHeight) * 3;
    GL_CALL(glGenBuffers(2, pbos_));
    for (unsigned int pbo : pbos_) {
        GL_CALL(glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo));
        GL_CALL(glBufferData(GL_PIXEL_PACK_BUFFER, static_cast<GLsizeiptr>(pboBytes), nullptr,
                              GL_STREAM_READ));
    }
    GL_CALL(glBindBuffer(GL_PIXEL_PACK_BUFFER, 0));

    byteSize_ = pboBytes * 3;  // 2 PBOs + the downsample texture, all the same size
    trackGpuAlloc(byteSize_);
}

Histogram::~Histogram() {
    destroy();
}

void Histogram::destroy() {
    if (downsampleFbo_ != 0) {
        trackGpuFree(byteSize_);
        byteSize_ = 0;
        glDeleteFramebuffers(1, &downsampleFbo_);
        glDeleteTextures(1, &downsampleTexture_);
        glDeleteBuffers(2, pbos_);
        downsampleFbo_ = 0;
        downsampleTexture_ = 0;
        pbos_[0] = 0;
        pbos_[1] = 0;
    }
}

Histogram::Histogram(Histogram&& other) noexcept
    : downsampleFbo_(std::exchange(other.downsampleFbo_, 0)),
      downsampleTexture_(std::exchange(other.downsampleTexture_, 0)),
      currentPbo_(other.currentPbo_),
      frameCounter_(other.frameCounter_),
      hasData_(other.hasData_),
      bins_(other.bins_),
      byteSize_(std::exchange(other.byteSize_, 0)) {
    pbos_[0] = std::exchange(other.pbos_[0], 0);
    pbos_[1] = std::exchange(other.pbos_[1], 0);
    pboWritten_[0] = other.pboWritten_[0];
    pboWritten_[1] = other.pboWritten_[1];
}

Histogram& Histogram::operator=(Histogram&& other) noexcept {
    if (this != &other) {
        destroy();
        downsampleFbo_ = std::exchange(other.downsampleFbo_, 0);
        downsampleTexture_ = std::exchange(other.downsampleTexture_, 0);
        pbos_[0] = std::exchange(other.pbos_[0], 0);
        pbos_[1] = std::exchange(other.pbos_[1], 0);
        currentPbo_ = other.currentPbo_;
        frameCounter_ = other.frameCounter_;
        pboWritten_[0] = other.pboWritten_[0];
        pboWritten_[1] = other.pboWritten_[1];
        hasData_ = other.hasData_;
        bins_ = other.bins_;
        byteSize_ = std::exchange(other.byteSize_, 0);
    }
    return *this;
}

void Histogram::update(int windowWidth, int windowHeight) {
    ++frameCounter_;
    if (frameCounter_ % kCaptureIntervalFrames != 0) {
        return;
    }

    // Downsample the just-composited default framebuffer into the fixed
    // small FBO.
    GL_CALL(glBindFramebuffer(GL_READ_FRAMEBUFFER, 0));
    GL_CALL(glBindFramebuffer(GL_DRAW_FRAMEBUFFER, downsampleFbo_));
    GL_CALL(glBlitFramebuffer(0, 0, windowWidth, windowHeight, 0, 0, kWidth, kHeight,
                              GL_COLOR_BUFFER_BIT, GL_LINEAR));

    // Kick off this capture's async readback into the current PBO --
    // non-blocking, since the target is a bound PBO, not client memory.
    GL_CALL(glBindFramebuffer(GL_READ_FRAMEBUFFER, downsampleFbo_));
    GL_CALL(glBindBuffer(GL_PIXEL_PACK_BUFFER, pbos_[currentPbo_]));
    GL_CALL(glReadPixels(0, 0, kWidth, kHeight, GL_RGB, GL_UNSIGNED_BYTE, nullptr));

    // Bin the *other* PBO's contents -- written a full capture interval
    // ago, so its transfer has long since completed and mapping it here
    // never stalls on this frame's GPU work.
    const int readyPbo = 1 - currentPbo_;
    if (pboWritten_[readyPbo]) {
        GL_CALL(glBindBuffer(GL_PIXEL_PACK_BUFFER, pbos_[readyPbo]));
        const auto* pixels = static_cast<const unsigned char*>(glMapBufferRange(
            GL_PIXEL_PACK_BUFFER, 0, static_cast<GLsizeiptr>(kWidth) * kHeight * 3,
            GL_MAP_READ_BIT));
        if (pixels != nullptr) {
            for (auto& channel : bins_) {
                channel.fill(0);
            }
            for (int i = 0; i < kWidth * kHeight; ++i) {
                ++bins_[0][pixels[(i * 3) + 0]];
                ++bins_[1][pixels[(i * 3) + 1]];
                ++bins_[2][pixels[(i * 3) + 2]];
            }
            GL_CALL(glUnmapBuffer(GL_PIXEL_PACK_BUFFER));
            hasData_ = true;
        }
    }

    pboWritten_[currentPbo_] = true;
    currentPbo_ = readyPbo;

    GL_CALL(glBindBuffer(GL_PIXEL_PACK_BUFFER, 0));
    GL_CALL(glBindFramebuffer(GL_READ_FRAMEBUFFER, 0));
    GL_CALL(glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0));
}

}  // namespace engine::debug
