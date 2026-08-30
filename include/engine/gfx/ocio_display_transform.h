#pragma once

#include <optional>

#include "engine/gfx/shader_program.h"

namespace engine::gfx {

// Owns three display shaders — sRGB LUT, Rec.1886/Rec.709 LUT, and a raw (unencoded) passthrough — compiled once at startup, switched at runtime via the debug 'L' key (cycles sRGB -> Rec709 -> Raw). Holds no OCIO::Const*RcPtr members: Config/Processor/GpuShaderDesc are only needed transiently in create() to generate GLSL text. No custom move semantics needed either — ShaderProgram is already move-only, and the rest are trivial scalars.
class OcioDisplayTransform {
public:
    enum class Lut { Raw, SRGB, Rec709 };

    // Builds all three shaders (see ocio_display_transform.cpp for the verified OCIO construction). Returns nullopt only on a GLSL compile/link failure (ShaderProgram's own recoverable-failure contract). An OCIO::Exception instead means this code is querying OCIO's fixed built-in registry incorrectly — an internal defect — and exits immediately, matching Window/HudOverlay's precedent.
    [[nodiscard]] static std::optional<OcioDisplayTransform> create();

    void setActiveLut(Lut lut) { activeLut_ = lut; }
    [[nodiscard]] Lut activeLut() const { return activeLut_; }

    // ev is a photographic stops adjustment; the GPU multiplier applied before the display curve (or, in Raw mode, before direct output) is pow(2, ev). Seeded from DebugCameraController::relativeExposureEv() (main.cpp), a relative-stops delta against profile.json's default aperture/shutter/ISO -- not an absolute photometric quantity, since the scene isn't calibrated to real-world radiance.
    void setExposureEv(float ev) { exposureEv_ = ev; }

    // 0 = off, 1/2/3 = isolate R/G/B (broadcast to grey), applied to the sampled texels before exposure. Uploaded by bind() alongside exposure, so switching channels is a uniform write rather than the full CPU re-copy and texture re-create it used to force.
    void setChannelView(int channelView) { channelView_ = channelView; }

    // 1.0 - rgb, applied to the final display-referred colour (after the display curve/Raw passthrough, before dither) -- the 'I' debug toggle.
    void setInvert(bool invert) { invert_ = invert; }

    // 0 = off. Radial per-channel UV offset (R pulled toward centre, B pushed away, G unchanged) applied at the texture fetch, before channel isolation/exposure/the display curve. Caller (main.cpp) is expected to pass 0 for any AOV other than Beauty.
    void setAberration(float aberration) { aberration_ = aberration; }

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

    // Uploads the exposure uniform to the active shader. Call once per frame, before PostProcessPass::draw consumes activeShader().
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
    int rawChannelViewLoc_;
    int srgbChannelViewLoc_;
    int rec709ChannelViewLoc_;
    int rawInvertLoc_;
    int srgbInvertLoc_;
    int rec709InvertLoc_;
    int rawAberrationLoc_;
    int srgbAberrationLoc_;
    int rec709AberrationLoc_;
    Lut activeLut_ = Lut::SRGB;
    float exposureEv_ = 0.0F;
    int channelView_ = 0;
    bool invert_ = false;
    float aberration_ = 0.0F;
};

}  // namespace engine::gfx
