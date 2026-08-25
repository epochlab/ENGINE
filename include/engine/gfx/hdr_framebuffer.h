#pragma once

#include <cstddef>

namespace engine::gfx {

// Owns one FBO with a GL_RGBA16F color texture attachment and a depth renderbuffer attachment, move-only. Depth is a renderbuffer, not a sampled texture — it's read back once per orbit-click via sampleDepth() (glReadPixels works against a bound renderbuffer attachment exactly like a texture one), not every frame, so a renderbuffer remains the simplest attachment that satisfies the actual need.
class HdrFramebuffer {
public:
    // width/height are framebuffer-pixel dimensions (e.g. from Window::framebufferSize()), not window-point size.
    HdrFramebuffer(int width, int height);
    ~HdrFramebuffer();

    HdrFramebuffer(const HdrFramebuffer&) = delete;
    HdrFramebuffer& operator=(const HdrFramebuffer&) = delete;
    HdrFramebuffer(HdrFramebuffer&& other) noexcept;
    HdrFramebuffer& operator=(HdrFramebuffer&& other) noexcept;

    // Destroys and recreates the color texture + depth renderbuffer at the new size (no-op if unchanged), re-checks completeness. Wired to Window's resize callback.
    void resize(int width, int height);

    void bind() const;

    [[nodiscard]] unsigned int colorTexture() const { return colorTexture_; }

    // Reads back the depth value at (x, y) in framebuffer pixels — for the debug orbit camera's click-to-pick pivot, not a per-frame call.
    [[nodiscard]] float sampleDepth(int x, int y) const;

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
