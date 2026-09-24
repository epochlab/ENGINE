#pragma once

#include <utility>

namespace pathtracer::gfx {

class ShaderProgram;

// Draws a fullscreen triangle (the gl_VertexID trick, no VBO) into the default framebuffer, sampling an HDR colour
// texture through whatever display shader is passed in.
class PostProcessPass {
public:
    PostProcessPass();
    ~PostProcessPass();

    PostProcessPass(const PostProcessPass&) = delete;
    PostProcessPass& operator=(const PostProcessPass&) = delete;
    PostProcessPass(PostProcessPass&& other) noexcept;
    PostProcessPass& operator=(PostProcessPass&& other) noexcept;

    // Binds framebuffer 0, sets the viewport to windowFramebufferSize, binds displayShader and hdrColorTexture on
    // unit 0, and draws the triangle.
    void draw(unsigned int hdrColorTexture, const ShaderProgram& displayShader,
              std::pair<int, int> windowFramebufferSize) const;

private:
    unsigned int vao_ = 0;
};

}  // namespace pathtracer::gfx
