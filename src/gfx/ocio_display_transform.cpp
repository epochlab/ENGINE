#include "engine/gfx/ocio_display_transform.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>

#include <GL/glew.h>

#include <OpenColorIO/OpenColorIO.h>

#include "engine/gfx/gl_debug.h"

namespace OCIO = OCIO_NAMESPACE;

namespace engine::gfx {

namespace {

// Pinned, not "-latest": empirically verified (compiled + ran against the installed library) that this config's "Un-tone-mapped" view is a pure colorimetric pass (0->0, 1->1, zero LUT textures), no filmic rolloff. Pinning avoids a future brew upgrade silently resolving "-latest" to a structurally different config.
constexpr const char* kBuiltinConfigName = "cg-config-v1.0.0_aces-v1.3_ocio-v2.1";
constexpr const char* kSceneColorSpace = "Linear Rec.709 (sRGB)";
constexpr const char* kView = "Un-tone-mapped";

// Channel isolation, applied to the sampled texels before exposure and the display curve -- the exact pipeline position the CPU-side bake occupied, so the displayed result is unchanged. As a uniform it costs no re-upload, which is the point: it previously forced a full re-copy and re-create of the display texture on every R/G/B keypress.
constexpr const char* kChannelViewGlsl =
    "uniform int uChannelView;\n";

constexpr const char* kApplyChannelViewGlsl =
    "    if (uChannelView == 1) { hdrColor = vec3(hdrColor.r); }\n"
    "    else if (uChannelView == 2) { hdrColor = vec3(hdrColor.g); }\n"
    "    else if (uChannelView == 3) { hdrColor = vec3(hdrColor.b); }\n";

// 1.0 - rgb, applied to the final display-referred colour (after the display curve/Raw passthrough, before dither).
constexpr const char* kInvertGlsl = "uniform bool uInvert;\n";

// Radial per-channel UV offset, 0 = off: R pulled toward centre, B pushed away, G unchanged -- the classic
// lens-chromatic-aberration look, strongest toward the frame edges since it scales with distance from
// centre. A post-process effect over Beauty only (main.cpp zeroes this for every other AOV), so it lives
// at the one place both LUTs and Raw already sample uHdrColor, rather than as a separate render pass.
constexpr const char* kAberrationGlsl =
    "uniform float uAberration;\n"
    "vec3 sampleAberrated(vec2 uv) {\n"
    "    vec2 dir = uv - vec2(0.5);\n"
    "    vec3 color;\n"
    "    color.r = texture(uHdrColor, uv - dir * uAberration).r;\n"
    "    color.g = texture(uHdrColor, uv).g;\n"
    "    color.b = texture(uHdrColor, uv + dir * uAberration).b;\n"
    "    return color;\n"
    "}\n";

// Triangular-PDF dither before the default framebuffer's 8-bit fixed-point quantization -- without it, smooth dark gradients in a converged (Monte Carlo noise no longer masking anything) render band visibly. Two independent uniform draws from a screen-space hash, subtracted for a triangular distribution in [-1/255, 1/255]. Deliberately static per pixel, not time-varying: this targets a converged image, not motion, so no frame/time uniform is threaded in for it.
constexpr const char* kDitherGlsl =
    "float ditherRand(vec2 co) { return fract(sin(dot(co, vec2(12.9898, 78.233))) * 43758.5453); }\n"
    "vec3 ditherOffset(vec2 uv) {\n"
    "    return vec3((ditherRand(uv) - ditherRand(uv + vec2(0.618, 0.618))) / 255.0);\n"
    "}\n";

std::optional<std::string> readFile(const std::string& path) {
    const std::ifstream file(path);
    if (!file) {
        std::cerr << "OcioDisplayTransform: failed to open " << path << '\n';
        return std::nullopt;
    }
    std::ostringstream contents;
    contents << file.rdbuf();
    return contents.str();
}

// Wraps OCIO's generated GLSL (uniform decls + helper functions + the named OCIODisplay*(vec4) function) in this project's client fragment program. Exposure is applied here, as a uniform pow(2,ev) multiply before the OCIO function is called, not through OCIO's own dynamic-property/uniform system.
std::string buildFragmentSource(const std::string& ocioShaderText, const std::string& functionName) {
    std::ostringstream src;
    src << "#version 410 core\n\n"
        << "in vec2 vUv;\n"
        << "out vec4 fragColor;\n\n"
        << "uniform sampler2D uHdrColor;\n"
        << "uniform float uExposure;\n"
        << kChannelViewGlsl << kInvertGlsl << kAberrationGlsl << "\n"
        << kDitherGlsl
        << ocioShaderText << "\n"
        << "void main() {\n"
        << "    vec3 hdrColor = sampleAberrated(vUv);\n"
        << kApplyChannelViewGlsl
        << "    vec4 exposed = vec4(hdrColor * uExposure, 1.0);\n"
        << "    vec3 displayColor = " << functionName << "(exposed).rgb;\n"
        << "    if (uInvert) { displayColor = 1.0 - displayColor; }\n"
        << "    fragColor = vec4(displayColor + ditherOffset(vUv), 1.0);\n"
        << "}\n";
    return src.str();
}

// No OCIO involved at all: exposure applied, then output directly with no display encode — lets 'L' cycle to a genuine unencoded state for direct comparison against the two LUTs, rather than only ever toggling between two encoded curves.
std::string buildRawFragmentSource() {
    return std::string("#version 410 core\n\n"
                       "in vec2 vUv;\n"
                       "out vec4 fragColor;\n\n"
                       "uniform sampler2D uHdrColor;\n"
                       "uniform float uExposure;\n") +
           kChannelViewGlsl + kInvertGlsl + kAberrationGlsl + kDitherGlsl +
           "\nvoid main() {\n"
           "    vec3 hdrColor = sampleAberrated(vUv);\n" +
           kApplyChannelViewGlsl +
           "    vec3 displayColor = hdrColor * uExposure;\n"
           "    if (uInvert) { displayColor = 1.0 - displayColor; }\n"
           "    fragColor = vec4(displayColor + ditherOffset(vUv), 1.0);\n"
           "}\n";
}

// Builds one LUT's fragment source via OCIO's real Display/View API (getProcessor(scene, display, view, direction)) using the "Un-tone-mapped" view — a genuine View entry on this config, not a colorspace-pair bypass. Both displays resolve to pure Matrix+Gamma OETF math (verified against OCIO's own source + empirical testing): no LUT ops, no filmic tone-mapping. OCIO::Exception here means our own config/display/view/style names are wrong — an internal defect, not recoverable input — so it exits immediately like Window/HudOverlay do for unrecoverable configuration errors, rather than threading optional through callers.
std::string buildOcioFragmentSource(const char* display, const char* functionName) {
    try {
        const OCIO::ConstConfigRcPtr config = OCIO::Config::CreateFromBuiltinConfig(kBuiltinConfigName);
        const OCIO::ConstProcessorRcPtr processor = config->getProcessor(
            kSceneColorSpace, display, kView, OCIO::TRANSFORM_DIR_FORWARD);
        const OCIO::ConstGPUProcessorRcPtr gpuProcessor = processor->getDefaultGPUProcessor();

        OCIO::GpuShaderDescRcPtr shaderDesc = OCIO::GpuShaderDesc::CreateShaderDesc();
        shaderDesc->setLanguage(OCIO::GPU_LANGUAGE_GLSL_4_0);
        shaderDesc->setFunctionName(functionName);
        gpuProcessor->extractGpuShaderInfo(shaderDesc);

        // Defensive, not merely documentary: this design assumes zero LUT textures (verified empirically against the installed OCIO 2.5.2). If a future OCIO/config change ever introduces one, silently skipping texture upload would render garbage; fail loudly instead so the assumption break is caught immediately.
        if (shaderDesc->getNumTextures() != 0 || shaderDesc->getNum3DTextures() != 0) {
            std::cerr << "OcioDisplayTransform: " << display
                      << " processor unexpectedly requires LUT textures ("
                      << shaderDesc->getNumTextures() << " 1D/2D, " << shaderDesc->getNum3DTextures()
                      << " 3D) — design assumption broken, aborting\n";
            std::exit(EXIT_FAILURE);
        }

        return buildFragmentSource(shaderDesc->getShaderText(), functionName);
    } catch (const OCIO::Exception& e) {
        std::cerr << "OcioDisplayTransform: OCIO error building " << display << ": " << e.what()
                   << '\n';
        std::exit(EXIT_FAILURE);
    }
}

}  // namespace

std::optional<OcioDisplayTransform> OcioDisplayTransform::create() {
    // Same fullscreen-triangle vertex shader for all three; only the fragment side varies (OCIO-generated for the two LUTs, hand-written for Raw).
    const std::optional<std::string> vertSrc =
        readFile(ASSET_ROOT_DIR "/shaders/fullscreen_triangle.vert");
    if (!vertSrc) {
        return std::nullopt;
    }

    const std::string rawFragSrc = buildRawFragmentSource();
    const std::string srgbFragSrc = buildOcioFragmentSource("sRGB - Display", "OCIODisplaySRGB");
    const std::string rec709FragSrc =
        buildOcioFragmentSource("Rec.1886 Rec.709 - Display", "OCIODisplayRec709");

    std::optional<ShaderProgram> rawShader = ShaderProgram::loadFromSource(*vertSrc, rawFragSrc);
    std::optional<ShaderProgram> srgbShader = ShaderProgram::loadFromSource(*vertSrc, srgbFragSrc);
    std::optional<ShaderProgram> rec709Shader =
        ShaderProgram::loadFromSource(*vertSrc, rec709FragSrc);
    if (!rawShader || !srgbShader || !rec709Shader) {
        return std::nullopt;
    }

    // One-time texture-unit assignment, matching main.cpp's existing explicit-uniform convention for uBaseColor/uHdrColor.
    rawShader->use();
    GL_CALL(glUniform1i(rawShader->uniformLocation("uHdrColor"), 0));
    srgbShader->use();
    GL_CALL(glUniform1i(srgbShader->uniformLocation("uHdrColor"), 0));
    rec709Shader->use();
    GL_CALL(glUniform1i(rec709Shader->uniformLocation("uHdrColor"), 0));

    return OcioDisplayTransform(std::move(*rawShader), std::move(*srgbShader),
                                 std::move(*rec709Shader));
}

OcioDisplayTransform::OcioDisplayTransform(ShaderProgram rawShader, ShaderProgram srgbShader,
                                            ShaderProgram rec709Shader)
    : rawShader_(std::move(rawShader)),
      srgbShader_(std::move(srgbShader)),
      rec709Shader_(std::move(rec709Shader)),
      rawExposureLoc_(rawShader_.uniformLocation("uExposure")),
      srgbExposureLoc_(srgbShader_.uniformLocation("uExposure")),
      rec709ExposureLoc_(rec709Shader_.uniformLocation("uExposure")),
      rawChannelViewLoc_(rawShader_.uniformLocation("uChannelView")),
      srgbChannelViewLoc_(srgbShader_.uniformLocation("uChannelView")),
      rec709ChannelViewLoc_(rec709Shader_.uniformLocation("uChannelView")),
      rawInvertLoc_(rawShader_.uniformLocation("uInvert")),
      srgbInvertLoc_(srgbShader_.uniformLocation("uInvert")),
      rec709InvertLoc_(rec709Shader_.uniformLocation("uInvert")),
      rawAberrationLoc_(rawShader_.uniformLocation("uAberration")),
      srgbAberrationLoc_(srgbShader_.uniformLocation("uAberration")),
      rec709AberrationLoc_(rec709Shader_.uniformLocation("uAberration")) {}

// Not wrapped in GL_CALL: runs every frame.
void OcioDisplayTransform::bind() const {
    const ShaderProgram& shader = activeShader();
    int exposureLoc = rawExposureLoc_;
    int channelViewLoc = rawChannelViewLoc_;
    int invertLoc = rawInvertLoc_;
    int aberrationLoc = rawAberrationLoc_;
    if (activeLut_ == Lut::SRGB) {
        exposureLoc = srgbExposureLoc_;
        channelViewLoc = srgbChannelViewLoc_;
        invertLoc = srgbInvertLoc_;
        aberrationLoc = srgbAberrationLoc_;
    } else if (activeLut_ == Lut::Rec709) {
        exposureLoc = rec709ExposureLoc_;
        channelViewLoc = rec709ChannelViewLoc_;
        invertLoc = rec709InvertLoc_;
        aberrationLoc = rec709AberrationLoc_;
    }
    shader.use();
    glUniform1f(exposureLoc, std::pow(2.0F, exposureEv_));
    glUniform1i(channelViewLoc, channelView_);
    glUniform1i(invertLoc, invert_ ? 1 : 0);
    glUniform1f(aberrationLoc, aberration_);
}

}  // namespace engine::gfx
