#include "engine/gfx/env_prefilter_pass.h"

#include <array>
#include <cstddef>
#include <utility>

#include <GL/glew.h>

#include <glm/glm.hpp>

#include "engine/gfx/gl_debug.h"
#include "engine/gfx/shader_program.h"
#include "engine/gfx/texture.h"

namespace engine::gfx {

namespace {

constexpr int kBaseFaceSize = 128;
constexpr int kMipCount = 6;

struct FaceBasis {
    glm::vec3 forward;
    glm::vec3 right;
    glm::vec3 up;
};

// GL_TEXTURE_CUBE_MAP_POSITIVE_X order: +X,-X,+Y,-Y,+Z,-Z. Derived from the OpenGL cubemap face direction spec (sc/tc/ma per face) so that a pixel written at this shader's (ndc.x, ndc.y) = (sc, tc) lands where a later texture(samplerCube, dir) read expects it -- matching sh_irradiance.h's direction convention (see equirect_to_cubemap.frag's header comment).
constexpr std::array<FaceBasis, 6> kFaceBases = {{
    {{1.0F, 0.0F, 0.0F}, {0.0F, 0.0F, -1.0F}, {0.0F, -1.0F, 0.0F}},
    {{-1.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, {0.0F, -1.0F, 0.0F}},
    {{0.0F, 1.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}},
    {{0.0F, -1.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, {0.0F, 0.0F, -1.0F}},
    {{0.0F, 0.0F, 1.0F}, {1.0F, 0.0F, 0.0F}, {0.0F, -1.0F, 0.0F}},
    {{0.0F, 0.0F, -1.0F}, {-1.0F, 0.0F, 0.0F}, {0.0F, -1.0F, 0.0F}},
}};

// One attribute-less VAO -- Apple's core-profile driver requires some VAO bound for any draw call even with zero vertex attributes (same rationale as PostProcessPass, not shared with it: different lifetime/usage, this one lives only for the duration of the bake).
unsigned int makeEmptyVao() {
    unsigned int vao = 0;
    GL_CALL(glGenVertexArrays(1, &vao));
    return vao;
}

void drawFullscreenTriangle(unsigned int vao) {
    GL_CALL(glBindVertexArray(vao));
    GL_CALL(glDrawArrays(GL_TRIANGLES, 0, 3));
}

void setFaceBasisUniforms(const ShaderProgram& shader, const FaceBasis& basis) {
    GL_CALL(glUniform3fv(shader.uniformLocation("uFaceRight"), 1, &basis.right[0]));
    GL_CALL(glUniform3fv(shader.uniformLocation("uFaceUp"), 1, &basis.up[0]));
    GL_CALL(glUniform3fv(shader.uniformLocation("uFaceForward"), 1, &basis.forward[0]));
}

}  // namespace

PrefilteredEnvironment buildPrefilteredEnvironment(const Texture& equirect,
                                                    const ShaderProgram& equirectToCubemapShader,
                                                    const ShaderProgram& prefilterShader) {
    CubemapTexture specular(kBaseFaceSize, kMipCount);

    unsigned int fbo = 0;
    GL_CALL(glGenFramebuffers(1, &fbo));
    const unsigned int vao = makeEmptyVao();

    GL_CALL(glDisable(GL_DEPTH_TEST));
    GL_CALL(glDisable(GL_BLEND));

    // Mip 0: direct equirect resample (roughness 0 -- prefiltering a delta BRDF would just reproduce the source at extra cost).
    equirectToCubemapShader.use();
    GL_CALL(glUniform1i(equirectToCubemapShader.uniformLocation("uEquirect"), 0));
    equirect.bind(0);
    for (int face = 0; face < 6; ++face) {
        setFaceBasisUniforms(equirectToCubemapShader, kFaceBases[static_cast<std::size_t>(face)]);
        specular.attachFaceForWrite(fbo, face, 0);
        drawFullscreenTriangle(vao);
    }

    // Mips 1..N-1: GGX-importance-sample-prefiltered from mip 0 of the same cubemap (prefilter_specular.frag reads it via an explicit LOD 0, since higher mips aren't populated yet mid-bake), roughness mapped linearly across the mip range.
    prefilterShader.use();
    GL_CALL(glUniform1i(prefilterShader.uniformLocation("uEnvCubemap"), 0));
    // Binds the same cubemap object that attachFaceForWrite below simultaneously attaches as this FBO's color target -- a texture feedback loop, undefined per spec without glTextureBarrier (core in GL 4.5, unavailable on this project's GL 4.1 core target). Safe in practice only because prefilter_specular.frag reads exclusively via an explicit textureLod(..., 0.0), and mip 0 is never one of the mips 1..N-1 being written this loop -- read and write mips never alias. Do not change the shader to sample any other LOD without re-deriving this.
    specular.bind(0);
    for (int mip = 1; mip < kMipCount; ++mip) {
        const float roughness = static_cast<float>(mip) / static_cast<float>(kMipCount - 1);
        GL_CALL(glUniform1f(prefilterShader.uniformLocation("uRoughness"), roughness));
        for (int face = 0; face < 6; ++face) {
            setFaceBasisUniforms(prefilterShader, kFaceBases[static_cast<std::size_t>(face)]);
            specular.attachFaceForWrite(fbo, face, mip);
            drawFullscreenTriangle(vao);
        }
    }

    GL_CALL(glBindFramebuffer(GL_FRAMEBUFFER, 0));
    GL_CALL(glDeleteFramebuffers(1, &fbo));
    GL_CALL(glDeleteVertexArrays(1, &vao));

    return PrefilteredEnvironment{std::move(specular), kMipCount};
}

}  // namespace engine::gfx
