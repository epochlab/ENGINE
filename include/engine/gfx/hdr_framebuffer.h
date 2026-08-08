#pragma once

#include <cstddef>

namespace engine::gfx {

// Owns one FBO with a GL_RGBA16F color texture attachment and a depth
// renderbuffer attachment, move-only. Depth is a renderbuffer, not a
// sampled texture: nothing in Phase 0 reads depth back (Phase 1's AOV
// selector is the first plausible consumer) — a renderbuffer is the
// simplest thing that satisfies the actual need.
class HdrFramebuffer {
public:
    // width/height are framebuffer-pixel dimensions (e.g. from
    // Window::framebufferSize()), not window-point size.
    HdrFramebuffer(int width, int height);
    ~HdrFramebuffer();

    HdrFramebuffer(const HdrFramebuffer&) = delete;
    HdrFramebuffer& operator=(const HdrFramebuffer&) = delete;
    HdrFramebuffer(HdrFramebuffer&& other) noexcept;
    HdrFramebuffer& operator=(HdrFramebuffer&& other) noexcept;

    // Destroys and recreates the color texture + depth renderbuffer at
    // the new size (no-op if unchanged), re-checks completeness. Wired to
    // Window's resize callback.
    void resize(int width, int height);

    void bind() const;

    [[nodiscard]] unsigned int colorTexture() const { return colorTexture_; }

private:
    void createAttachments(int width, int height);
    void destroyAttachments();

    unsigned int fbo_ = 0;
    unsigned int colorTexture_ = 0;
    unsigned int depthRenderbuffer_ = 0;
    int width_ = 0;
    int height_ = 0;
    std::size_t byteSize_ = 0;  // reported to engine::debug's GPU memory tracker
};

}  // namespace engine::gfx
