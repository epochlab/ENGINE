#pragma once

#include <utility>

namespace engine::gfx {

class ShaderProgram;

// Draws a fullscreen triangle (gl_VertexID trick, no VBO) into the default framebuffer, sampling an HDR color texture through whatever display shader is passed in. Owns exactly one attribute-less VAO, created once — Apple's core-profile driver requires *some* VAO bound for any draw call, even with zero vertex attributes.
class PostProcessPass {
public:
    PostProcessPass();
    ~PostProcessPass();

    PostProcessPass(const PostProcessPass&) = delete;
    PostProcessPass& operator=(const PostProcessPass&) = delete;
    PostProcessPass(PostProcessPass&& other) noexcept;
    PostProcessPass& operator=(PostProcessPass&& other) noexcept;

    // Binds framebuffer 0, sets the viewport to windowFramebufferSize, binds displayShader + hdrColorTexture (texture unit 0), draws the fullscreen triangle. displayShader/hdrColorTexture are passed in rather than owned: Stage F swaps the active display shader (sRGB/Rec.709) at runtime without this class needing to know.
    void draw(unsigned int hdrColorTexture, const ShaderProgram& displayShader,
              std::pair<int, int> windowFramebufferSize) const;

private:
    unsigned int vao_ = 0;
};

}  // namespace engine::gfx
