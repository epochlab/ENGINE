#include "pathtracer/gfx/ocio_display_transform.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>

#include <GL/glew.h>

#include <OpenColorIO/OpenColorIO.h>

#include "pathtracer/gfx/gl_debug.h"

namespace OCIO = OCIO_NAMESPACE;

namespace pathtracer::gfx {

namespace {

// Declared in ocio_display_transform.h so CPU-side consumers reproduce this exact transform -- see the header.
constexpr const char* kBuiltinConfigName = kOcioConfigName;
constexpr const char* kSceneColorSpace = kOcioSceneColorSpace;
constexpr const char* kView = kOcioView;

// Channel isolation, applied to the sampled texels before exposure and the display curve -- the exact pipeline
// position the CPU-side bake occupied, so the displayed result is unchanged. As a uniform it costs no re-upload.
constexpr const char* kChannelViewGlsl =
    "uniform int uChannelView;\n";

constexpr const char* kApplyChannelViewGlsl =
    "    if (uChannelView == 1) { hdrColor = vec3(hdrColor.r); }\n"
    "    else if (uChannelView == 2) { hdrColor = vec3(hdrColor.g); }\n"
    "    else if (uChannelView == 3) { hdrColor = vec3(hdrColor.b); }\n";

// 1.0 - rgb, applied to the final display-referred colour (after the display curve/Raw passthrough, before dither).
constexpr const char* kInvertGlsl = "uniform bool uInvert;\n";

// Radial per-channel UV offset, 0 = off: R pulled toward centre, B pushed away, G unchanged, strongest at the frame
// edges since it scales with distance from centre. Beauty only, so it lives where both LUTs and Raw already sample
// uHdrColor rather than as a separate render pass.
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

// Triangular-PDF dither before the default framebuffer's 8-bit quantization: without it, smooth dark gradients in a
// converged render band visibly, Monte Carlo noise no longer masking them.
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

// Wraps OCIO's generated GLSL in this project's client fragment program. Exposure is applied here as a uniform
// pow(2,ev) multiply before the OCIO function, so it stays scene-referred.
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

// No OCIO at all: exposure applied, then output directly with no display encode. Lets 'L' cycle to a genuinely
// unencoded state for direct comparison, rather than only ever toggling between two encoded ones.
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

// Builds one LUT's fragment source through OCIO's real Display/View API using the "Un-tone-mapped" view, a genuine
// View entry rather than a colorspace-pair bypass. Both displays resolve to pure Matrix+Gamma OETF math, verified
// against OCIO source and empirically. A config/display/view name error is an internal defect, so it exits at once.
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

        // Defensive, not merely documentary: this design assumes zero LUT textures, verified against the installed
        // OCIO. If a future config ever introduces one, silently skipping the upload would render wrong colours.
        if (shaderDesc->getNumTextures() != 0 || shaderDesc->getNum3DTextures() != 0) {
            std::cerr << "OcioDisplayTransform: " << display
                      << " processor unexpectedly requires LUT textures ("
                      << shaderDesc->getNumTextures() << " 1D/2D, " << shaderDesc->getNum3DTextures()
                      << " 3D): design assumption broken, aborting\n";
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
    // Same fullscreen-triangle vertex shader for all three; only the fragment side varies.
    const std::optional<std::string> vertSrc =
        readFile(ASSET_ROOT_DIR "/shaders/fullscreen_triangle.vert");
    if (!vertSrc) {
        return std::nullopt;
    }

    const std::string rawFragSrc = buildRawFragmentSource();
    const std::string srgbFragSrc = buildOcioFragmentSource(kOcioSrgbDisplay, "OCIODisplaySRGB");
    const std::string rec709FragSrc =
        buildOcioFragmentSource(kOcioRec709Display, "OCIODisplayRec709");

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

}  // namespace pathtracer::gfx
