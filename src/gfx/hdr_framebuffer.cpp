#include "engine/gfx/hdr_framebuffer.h"

#include <cstdlib>
#include <iostream>
#include <utility>

#include <GL/glew.h>

#include "engine/gfx/gl_debug.h"

namespace engine::gfx {

namespace {

void checkCompleteness() {
    const GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "HdrFramebuffer: incomplete, status=0x" << std::hex << status << std::dec
                  << '\n';
        // Fixed, code-controlled attachment config, not external input —
        // a real programming defect, matching Window's precedent (log +
        // std::exit on unrecoverable setup failure; no exceptions
        // anywhere in this codebase).
        std::exit(EXIT_FAILURE);
    }
}

}  // namespace

HdrFramebuffer::HdrFramebuffer(int width, int height) {
    GL_CALL(glGenFramebuffers(1, &fbo_));
    createAttachments(width, height);
}

HdrFramebuffer::~HdrFramebuffer() {
    destroyAttachments();
    if (fbo_ != 0) {
        glDeleteFramebuffers(1, &fbo_);
    }
}

HdrFramebuffer::HdrFramebuffer(HdrFramebuffer&& other) noexcept
    : fbo_(std::exchange(other.fbo_, 0)),
      colorTexture_(std::exchange(other.colorTexture_, 0)),
      depthRenderbuffer_(std::exchange(other.depthRenderbuffer_, 0)),
      width_(std::exchange(other.width_, 0)),
      height_(std::exchange(other.height_, 0)) {}

HdrFramebuffer& HdrFramebuffer::operator=(HdrFramebuffer&& other) noexcept {
    if (this != &other) {
        destroyAttachments();
        if (fbo_ != 0) {
            glDeleteFramebuffers(1, &fbo_);
        }
        fbo_ = std::exchange(other.fbo_, 0);
        colorTexture_ = std::exchange(other.colorTexture_, 0);
        depthRenderbuffer_ = std::exchange(other.depthRenderbuffer_, 0);
        width_ = std::exchange(other.width_, 0);
        height_ = std::exchange(other.height_, 0);
    }
    return *this;
}

void HdrFramebuffer::createAttachments(int width, int height) {
    width_ = width;
    height_ = height;

    GL_CALL(glBindFramebuffer(GL_FRAMEBUFFER, fbo_));

    GL_CALL(glGenTextures(1, &colorTexture_));
    GL_CALL(glBindTexture(GL_TEXTURE_2D, colorTexture_));
    GL_CALL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGBA, GL_FLOAT,
                          nullptr));
    GL_CALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
    GL_CALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    GL_CALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    GL_CALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
    GL_CALL(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                                    colorTexture_, 0));

    // 24-bit fixed-point, not 32F: depth test isn't even enabled this
    // stage (one quad, nothing to sort) and depth is never sampled back
    // (renderbuffer, not texture, per the design note above) — 24-bit is
    // the universal baseline; swapping to 32F later is a one-line change
    // if a real precision need appears.
    GL_CALL(glGenRenderbuffers(1, &depthRenderbuffer_));
    GL_CALL(glBindRenderbuffer(GL_RENDERBUFFER, depthRenderbuffer_));
    GL_CALL(glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height));
    GL_CALL(glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER,
                                       depthRenderbuffer_));

    checkCompleteness();

    GL_CALL(glBindFramebuffer(GL_FRAMEBUFFER, 0));
    GL_CALL(glBindTexture(GL_TEXTURE_2D, 0));
    GL_CALL(glBindRenderbuffer(GL_RENDERBUFFER, 0));
}

void HdrFramebuffer::destroyAttachments() {
    if (colorTexture_ != 0) {
        glDeleteTextures(1, &colorTexture_);
        colorTexture_ = 0;
    }
    if (depthRenderbuffer_ != 0) {
        glDeleteRenderbuffers(1, &depthRenderbuffer_);
        depthRenderbuffer_ = 0;
    }
}

void HdrFramebuffer::resize(int width, int height) {
    if (width == width_ && height == height_) {
        return;
    }
    destroyAttachments();
    createAttachments(width, height);
}

// Not wrapped in GL_CALL: runs every frame.
void HdrFramebuffer::bind() const {
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glViewport(0, 0, width_, height_);
}

}  // namespace engine::gfx
