#pragma once

#include <optional>

#include "pathtracer/gfx/shader_program.h"

namespace pathtracer::gfx {

// The colour pipeline's definition, exposed so a CPU-side consumer reproduces the exact transform the viewer displays.
// kBuiltinConfigName is pinned, not "-latest": verified by running against the installed library that this config's
// "Un-tone-mapped" view is a pure colorimetric pass (0->0, 1->1, zero crosstalk).
inline constexpr const char* kOcioConfigName = "cg-config-v1.0.0_aces-v1.3_ocio-v2.1";
inline constexpr const char* kOcioSceneColorSpace = "Linear Rec.709 (sRGB)";
inline constexpr const char* kOcioView = "Un-tone-mapped";
inline constexpr const char* kOcioSrgbDisplay = "sRGB - Display";
inline constexpr const char* kOcioRec709Display = "Rec.1886 Rec.709 - Display";

// Owns three display shaders: sRGB LUT, Rec.1886/Rec.709 LUT, and a raw unencoded passthrough. Compiled once at
// startup, switched at runtime by the debug 'L' key (sRGB -> Rec709 -> Raw).
class OcioDisplayTransform {
public:
    enum class Lut { Raw, SRGB, Rec709 };

    // Builds all three shaders. Returns nullopt only on a GLSL compile or link failure, per ShaderProgram's contract.
    [[nodiscard]] static std::optional<OcioDisplayTransform> create();

    void setActiveLut(Lut lut) { activeLut_ = lut; }
    [[nodiscard]] Lut activeLut() const { return activeLut_; }

    // ev is a stops adjustment; the GPU multiplier applied before the display curve is pow(2, ev). Seeded from
    // DebugCameraController's relative exposure.
    void setExposureEv(float ev) { exposureEv_ = ev; }

    // 0 = off, 1/2/3 isolate R/G/B broadcast to grey, applied before exposure. Uploaded by bind() alongside exposure,
    // so switching channels is a uniform write rather than a shader swap.
    void setChannelView(int channelView) { channelView_ = channelView; }

    // 1.0 - rgb on the final display-referred colour, after the display curve and before dither -- the 'I' toggle.
    void setInvert(bool invert) { invert_ = invert; }

    // 0 = off. Radial per-channel UV offset (R toward centre, B away, G unchanged) at the texture fetch, before
    // channel isolation, exposure and the display curve.
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

}  // namespace pathtracer::gfx
