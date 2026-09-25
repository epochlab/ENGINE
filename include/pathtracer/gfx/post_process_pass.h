#pragma once

#include "pathtracer/gfx/viewport.h"

namespace pathtracer::gfx {

class ShaderProgram;

// Draws a fullscreen triangle (the gl_VertexID trick, no VBO) into the default framebuffer, sampling an HDR colour texture.
class PostProcessPass {
public:
    PostProcessPass();
    ~PostProcessPass();

    PostProcessPass(const PostProcessPass&) = delete;
    PostProcessPass& operator=(const PostProcessPass&) = delete;
    PostProcessPass(PostProcessPass&& other) noexcept;
    PostProcessPass& operator=(PostProcessPass&& other) noexcept;

    // Binds framebuffer 0, sets the viewport to imageRect, binds displayShader and hdrColorTexture on unit 0, draws. Never clears.
    void draw(unsigned int hdrColorTexture, const ShaderProgram& displayShader, ViewportRect imageRect) const;

private:
    unsigned int vao_ = 0;
};

}  // namespace pathtracer::gfx
