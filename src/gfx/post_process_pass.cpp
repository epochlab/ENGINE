#include "engine/gfx/post_process_pass.h"

#include <utility>

#include <GL/glew.h>

#include "engine/gfx/gl_debug.h"
#include "engine/gfx/shader_program.h"

namespace engine::gfx {

PostProcessPass::PostProcessPass() {
    GL_CALL(glGenVertexArrays(1, &vao_));
}

PostProcessPass::~PostProcessPass() {
    if (vao_ != 0) {
        glDeleteVertexArrays(1, &vao_);
    }
}

PostProcessPass::PostProcessPass(PostProcessPass&& other) noexcept
    : vao_(std::exchange(other.vao_, 0)) {}

PostProcessPass& PostProcessPass::operator=(PostProcessPass&& other) noexcept {
    if (this != &other) {
        if (vao_ != 0) {
            glDeleteVertexArrays(1, &vao_);
        }
        vao_ = std::exchange(other.vao_, 0);
    }
    return *this;
}

// Not wrapped in GL_CALL: the entire body runs every frame.
void PostProcessPass::draw(unsigned int hdrColorTexture, const ShaderProgram& displayShader,
                            std::pair<int, int> windowFramebufferSize) const {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, windowFramebufferSize.first, windowFramebufferSize.second);

    displayShader.use();
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, hdrColorTexture);

    glBindVertexArray(vao_);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);
}

}  // namespace engine::gfx
