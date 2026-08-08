#pragma once

#include <optional>

#include "engine/gfx/shader_program.h"

namespace engine::gfx {

// Owns three display shaders — sRGB LUT, Rec.1886/Rec.709 LUT, and a raw
// (unencoded) passthrough — compiled once at startup, switched at runtime
// via the debug 'L' key (cycles sRGB -> Rec709 -> Raw).
//
// Holds no OCIO::Const*RcPtr members: Config/Processor/GpuShaderDesc are
// only needed transiently in create() to generate GLSL text. No custom
// move semantics needed either — ShaderProgram is already move-only, and
// the rest are trivial scalars.
class OcioDisplayTransform {
public:
    enum class Lut { Raw, SRGB, Rec709 };

    // Builds all three shaders (see ocio_display_transform.cpp for the
    // verified OCIO construction). Returns nullopt only on a GLSL
    // compile/link failure (ShaderProgram's own recoverable-failure
    // contract). An OCIO::Exception instead means this code is querying
    // OCIO's fixed built-in registry incorrectly — an internal defect —
    // and exits immediately, matching Window/HdrFramebuffer's precedent.
    [[nodiscard]] static std::optional<OcioDisplayTransform> create();

    void setActiveLut(Lut lut) { activeLut_ = lut; }
    [[nodiscard]] Lut activeLut() const { return activeLut_; }

    // ev is a photographic stops adjustment; the GPU multiplier applied
    // before the display curve (or, in Raw mode, before direct output) is
    // pow(2, ev). See camera.h's ev100()/exposure() model and main.cpp for
    // why this isn't currently seeded from Camera::exposure().
    void setExposureEv(float ev) { exposureEv_ = ev; }

    [[nodiscard]] const ShaderProgram& activeShader() const {
        switch (activeLut_) {
            case Lut::SRGB:
                return srgbShader_;
            case Lut::Rec709:
                return rec709Shader_;
            case Lut::Raw:
            default:
                return rawShader_;
        }
    }

    // Uploads the exposure uniform to the active shader. Call once per
    // frame, before PostProcessPass::draw consumes activeShader().
    void bind() const;

private:
    OcioDisplayTransform(ShaderProgram rawShader, ShaderProgram srgbShader,
                          ShaderProgram rec709Shader);

    ShaderProgram rawShader_;
    ShaderProgram srgbShader_;
    ShaderProgram rec709Shader_;
    int rawExposureLoc_;
    int srgbExposureLoc_;
    int rec709ExposureLoc_;
    Lut activeLut_ = Lut::SRGB;
    float exposureEv_ = 0.0F;
};

}  // namespace engine::gfx
