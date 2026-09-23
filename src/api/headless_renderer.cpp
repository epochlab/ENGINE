#include "engine/api/headless_renderer.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <optional>
#include <utility>

#include <glm/gtc/matrix_transform.hpp>

#include "engine/debug/aov_filters.h"
#include "engine/debug/aov_routing.h"
#include "engine/debug/render_stats.h"
#include "engine/scene/material_binding.h"

namespace engine::api {

namespace {

using engine::debug::AovId;
using engine::debug::AovSource;
using engine::gfx::HdrImage;

// Scene-level placement, order X,Y,Z -- must stay identical to main.cpp's and render_beauty's composition or a headless render places the scene differently from the viewer.
[[nodiscard]] glm::mat4 rootTransformOf(const engine::config::SceneConfig& scene) {
    return glm::translate(glm::mat4(1.0F), scene.model.position) *
           glm::rotate(glm::mat4(1.0F), glm::radians(scene.model.rotation.z), glm::vec3(0.0F, 0.0F, 1.0F)) *
           glm::rotate(glm::mat4(1.0F), glm::radians(scene.model.rotation.y), glm::vec3(0.0F, 1.0F, 0.0F)) *
           glm::rotate(glm::mat4(1.0F), glm::radians(scene.model.rotation.x), glm::vec3(1.0F, 0.0F, 0.0F));
}

// samplesPerPixel is 1 here and convergence comes from accumulating passes, which is how the progressive driver and render_beauty both drive the integrator; the field is left in PathTraceSettings for the viewer's single-shot mode.
[[nodiscard]] engine::scene::PathTraceSettings baseSettingsOf(const engine::config::ProfileConfig& profile,
                                                               const engine::config::MaterialConfig& material) {
    return engine::scene::PathTraceSettings{
        .samplesPerPixel = 1,
        .maxBounces = profile.pathTracer.maxBounces,
        .russianRouletteStartBounce = profile.pathTracer.russianRouletteStartBounce,
        .aoMaxDistance = profile.pathTracer.aoMaxDistance,
        .lookaheadDistance = profile.pathTracer.lookaheadDistance,
        .bumpStrength = material.bumpStrength,
        .roughnessMin = material.roughnessMin,
        .roughnessMax = material.roughnessMax,
        .diffuseColour = material.diffuseColour,
        .ior = material.ior,
        .abbe = material.abbe,
        .transmissionFactor = material.transmissionFactor,
        .metallicFactor = material.metallicFactor,
        .roughnessFactor = material.roughnessFactor,
        .diffuseRoughness = material.diffuseRoughness,
        .transmissionColor = material.transmissionColor,
        .transmissionDepth = material.transmissionDepth,
        .edgeTint = material.edgeTint,
    };
}

// profile.json names a film-back preset; assets/config/camera.json supplies its dimensions. Resolved exactly as main.cpp's initializeApp and render_beauty do.
[[nodiscard]] std::optional<engine::scene::Camera> resolveCamera(const std::string& assetRoot,
                                                                 const engine::config::ProfileConfig& profile,
                                                                 std::string& error) {
    const std::optional<std::vector<engine::scene::Camera::FilmBackPreset>> presets =
        engine::config::loadFilmBackPresets(assetRoot + "/config/camera.json");
    if (!presets) {
        error = "failed to load " + assetRoot + "/config/camera.json";
        return std::nullopt;
    }
    const auto preset = std::find_if(presets->begin(), presets->end(),
                                      [&](const engine::scene::Camera::FilmBackPreset& candidate) {
                                          return candidate.name == profile.camera.defaultFilmBackPresetName;
                                      });
    if (preset == presets->end()) {
        error = "profile.json filmBackPreset \"" + profile.camera.defaultFilmBackPresetName +
                "\" not found in camera.json";
        return std::nullopt;
    }
    const engine::config::CameraConfig& camera = profile.camera;
    return engine::scene::Camera(camera.position, camera.yawDegrees, camera.pitchDegrees, preset->filmBack,
                                  camera.focalLengthMm, camera.nearClip, camera.farClip, camera.aperture,
                                  camera.shutterSeconds, camera.iso);
}

// Gathers `channels` of each texel out of HdrImage's fixed RGBA layout into a tightly packed destination. The one copy the boundary costs: a scalar AOV is stored broadcast to three channels, and a consumer reading the data wants the one that carries it.
void packChannels(const HdrImage& source, int channels, float* destination,
                  engine::scene::ThreadPool& threadPool) {
    const int width = source.width;
    threadPool.parallelFor(source.height, [&](int y) {
        const std::size_t row = static_cast<std::size_t>(y) * static_cast<std::size_t>(width);
        for (int x = 0; x < width; ++x) {
            const std::size_t pixel = row + static_cast<std::size_t>(x);
            for (int c = 0; c < channels; ++c) {
                destination[(pixel * static_cast<std::size_t>(channels)) + static_cast<std::size_t>(c)] =
                    source.rgba[(pixel * 4) + static_cast<std::size_t>(c)];
            }
        }
    });
}

}  // namespace

std::unique_ptr<HeadlessRenderer> HeadlessRenderer::open(const std::string& assetRoot,
                                                          const std::string& scenePath,
                                                          std::string& error) {
    const std::optional<engine::config::ProfileConfig> profile =
        engine::config::loadProfileConfig(assetRoot + "/config/profile.json");
    if (!profile) {
        error = "failed to load " + assetRoot + "/config/profile.json";
        return nullptr;
    }
    const std::optional<engine::config::SceneConfig> scene =
        engine::config::loadSceneConfig(assetRoot + "/" + scenePath);
    if (!scene) {
        error = "failed to load scene " + assetRoot + "/" + scenePath;
        return nullptr;
    }
    const std::optional<engine::config::MaterialConfig> material =
        engine::config::loadMaterialConfig(assetRoot + "/" + scene->materialPath);
    if (!material) {
        error = "failed to load material " + assetRoot + "/" + scene->materialPath;
        return nullptr;
    }
    std::optional<engine::gfx::ImageTexture> environmentImage = engine::gfx::loadImageTexture(
        assetRoot + "/" + scene->environment.hdriPath, profile->render.textureType);
    if (!environmentImage) {
        error = "failed to load environment " + assetRoot + "/" + scene->environment.hdriPath;
        return nullptr;
    }

    const glm::mat4 rootTransform = rootTransformOf(*scene);
    std::optional<engine::scene::LoadedModel> model = engine::scene::loadGltf(
        assetRoot + "/" + scene->model.gltfPath, profile->render.textureType, rootTransform,
        scene->model.texturePath.empty() ? "" : assetRoot + "/" + scene->model.texturePath);
    if (!model) {
        error = "failed to load glTF " + assetRoot + "/" + scene->model.gltfPath;
        return nullptr;
    }

    std::vector<int> instanceLightIndex(model->instances.size(), -1);
    std::vector<engine::scene::QuadLight> quadLights =
        engine::scene::buildQuadLights(scene->lights, rootTransform);
    engine::scene::appendQuadLights(*model, quadLights, instanceLightIndex);

    std::optional<engine::scene::Camera> camera = resolveCamera(assetRoot, *profile, error);
    if (!camera) {
        return nullptr;
    }

    const engine::scene::PathTraceSettings baseSettings = baseSettingsOf(*profile, *material);
    std::optional<std::vector<engine::scene::PathTraceSettings>> perInstanceSettings =
        engine::scene::resolvePerInstanceSettings(baseSettings, model->instances,
                                                   scene->materialOverrides, assetRoot);
    if (!perInstanceSettings) {
        error = "failed to resolve per-instance material overrides";
        return nullptr;
    }

    std::optional<engine::scene::EmbreeAccel> accel =
        engine::scene::EmbreeAccel::build(std::move(model->worldTriangles));
    if (!accel) {
        error = "Embree scene build failed";
        return nullptr;
    }

    return std::unique_ptr<HeadlessRenderer>(new HeadlessRenderer(
        *profile, std::move(*model), std::move(quadLights), std::move(instanceLightIndex),
        std::move(*perInstanceSettings), baseSettings, std::move(*accel), std::move(*environmentImage),
        scene->environment.lightEnabled, *camera));
}

HeadlessRenderer::HeadlessRenderer(engine::config::ProfileConfig profile,
                                    engine::scene::LoadedModel model,
                                    std::vector<engine::scene::QuadLight> quadLights,
                                    std::vector<int> instanceLightIndex,
                                    std::vector<engine::scene::PathTraceSettings> perInstanceSettings,
                                    engine::scene::PathTraceSettings baseSettings,
                                    engine::scene::EmbreeAccel accel,
                                    engine::gfx::ImageTexture environmentImage, bool envLightEnabled,
                                    engine::scene::Camera defaultCamera)
    : profile_(std::move(profile)),
      model_(std::move(model)),
      instanceLightIndex_(std::move(instanceLightIndex)),
      perInstanceSettings_(std::move(perInstanceSettings)),
      baseSettings_(baseSettings),
      instanceBounds_(engine::scene::computeInstanceBounds(model_.shadingTriangles, static_cast<int>(model_.instances.size()))),
      accel_(std::move(accel)),
      environmentMap_(std::move(environmentImage)),
      quadLights_(std::move(quadLights)),
      lights_(&environmentMap_, /*envRotationRadians=*/0.0F, /*envExposure=*/1.0F, quadLights_),
      lightsEnvOff_(nullptr, /*envRotationRadians=*/0.0F, /*envExposure=*/1.0F, quadLights_),
      defaultEnvLightEnabled_(envLightEnabled),
      defaultCamera_(defaultCamera) {}

HeadlessRenderer::~HeadlessRenderer() = default;

void HeadlessRenderer::resizeBuffers(int width, int height) {
    if (bufferWidth_ == width && bufferHeight_ == height) {
        return;
    }
    pathTraced_ = engine::scene::makePathTraceResult(width, height);
    // renderRasterGBuffer reallocates its own 14 images when the size changes and clears them per row otherwise; resetting the generation stamp is what tells it this buffer holds nothing yet.
    gbuffer_ = engine::scene::RasterGBuffer{};
    accumulators_.clear();
    bufferWidth_ = width;
    bufferHeight_ = height;
}

const engine::gfx::HdrImage& HeadlessRenderer::lastImage(AovId aov) const {
    switch (engine::debug::aovSource(aov)) {
        case AovSource::PathTraced: {
            const auto it = std::find(accumulatedAovs_.begin(), accumulatedAovs_.end(), aov);
            return accumulators_[static_cast<std::size_t>(it - accumulatedAovs_.begin())];
        }
        case AovSource::GBuffer:
            return gbuffer_.*engine::debug::gbufferLane(aov);
        case AovSource::BeautyFilter: {
            const auto it = std::find(filteredAovs_.begin(), filteredAovs_.end(), aov);
            return filtered_[static_cast<std::size_t>(it - filteredAovs_.begin())];
        }
    }
    return gbuffer_.depth;
}

bool HeadlessRenderer::render(const Request& request, std::span<float* const> outputs,
                               std::string& error) {
    if (outputs.size() != request.aovs.size()) {
        error = "one output buffer is required per requested AOV";
        return false;
    }
    if (!render(request, error)) {
        return false;
    }
    for (std::size_t i = 0; i < request.aovs.size(); ++i) {
        packChannels(lastImage(request.aovs[i]), engine::debug::aovChannels(request.aovs[i]), outputs[i],
                     threadPool_);
    }
    return true;
}

bool HeadlessRenderer::render(const Request& request, std::string& error) {
    if (request.width <= 0 || request.height <= 0) {
        error = "resolution must be positive";
        return false;
    }
    if (request.samples <= 0) {
        error = "samples must be at least 1";
        return false;
    }
    if (request.aovs.empty()) {
        error = "no AOVs requested";
        return false;
    }
    resizeBuffers(request.width, request.height);

    const bool wantsFilter = std::any_of(request.aovs.begin(), request.aovs.end(), [](AovId aov) {
        return engine::debug::aovSource(aov) == AovSource::BeautyFilter;
    });
    const bool wantsGBuffer = std::any_of(request.aovs.begin(), request.aovs.end(), [](AovId aov) {
        return engine::debug::aovSource(aov) == AovSource::GBuffer;
    });

    // The path-traced lanes this request needs summed. Beauty joins the set whenever a filter is asked for, because every filter reads the accumulated Beauty rather than a per-pass one.
    accumulatedAovs_.clear();
    for (const AovId aov : request.aovs) {
        if (engine::debug::aovSource(aov) == AovSource::PathTraced &&
            std::find(accumulatedAovs_.begin(), accumulatedAovs_.end(), aov) == accumulatedAovs_.end()) {
            accumulatedAovs_.push_back(aov);
        }
    }
    if (wantsFilter && std::find(accumulatedAovs_.begin(), accumulatedAovs_.end(), AovId::Beauty) ==
                            accumulatedAovs_.end()) {
        accumulatedAovs_.push_back(AovId::Beauty);
    }

    stats_.passMilliseconds.clear();
    stats_.rasterMilliseconds = 0.0;
    stats_.filterMilliseconds = 0.0;
    stats_.rays = engine::debug::RayCounts{};
    if (!accumulatedAovs_.empty()) {
        accumulators_.assign(accumulatedAovs_.size(),
                              engine::gfx::HdrImage{request.width, request.height,
                                                     std::vector<float>(static_cast<std::size_t>(request.width) *
                                                                            static_cast<std::size_t>(request.height) * 4,
                                                                        0.0F)});
        // A direct synchronous caller does not use PathTraceDriver's cooperative cancellation, so generation is held at the value requestedGeneration asks for and never goes stale.
        const std::atomic<std::uint64_t> generation{1};
        engine::debug::PassStats stats;
        const engine::scene::LightSet& lights =
            request.envLightEnabled.value_or(defaultEnvLightEnabled_) ? lights_ : lightsEnvOff_;
        stats_.passMilliseconds.reserve(static_cast<std::size_t>(request.samples));
        for (int pass = 0; pass < request.samples; ++pass) {
            // Only the trace is timed: the accumulation below it is O(pixels) and identical across revisions.
            const auto passStart = std::chrono::steady_clock::now();
            // scrambleSeed fixed, sampleBase advancing: the pair that keeps the accumulated samples an Owen-scrambled Sobol sequence stratified against each other rather than independent draws (sampler.h).
            engine::scene::renderPathTraced(request.camera, accel_, model_.shadingTriangles, model_.instances,
                                             instanceLightIndex_, lights, request.width, request.height,
                                             /*showSky=*/true, baseSettings_, perInstanceSettings_,
                                             request.scrambleSeed, /*sampleBase=*/pass,
                                             /*sampleCount=*/request.samples, generation,
                                             /*requestedGeneration=*/1U, threadPool_, stats, pathTraced_);
            stats_.passMilliseconds.push_back(
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - passStart).count());
            for (std::size_t lane = 0; lane < accumulatedAovs_.size(); ++lane) {
                const engine::debug::PathTracedLane member = engine::debug::pathTracedLane(accumulatedAovs_[lane]);
                const std::vector<float>& source = (pathTraced_.*member).rgba;
                std::vector<float>& sum = accumulators_[lane].rgba;
                for (std::size_t i = 0; i < sum.size(); ++i) {
                    sum[i] += source[i];
                }
            }
        }
        stats_.rays = stats.rays();
        const auto passes = static_cast<float>(request.samples);
        for (engine::gfx::HdrImage& accumulator : accumulators_) {
            for (float& value : accumulator.rgba) {
                value /= passes;
            }
        }
    }

    if (wantsGBuffer) {
        const auto rasterStart = std::chrono::steady_clock::now();
        engine::scene::renderRasterGBuffer(request.camera, model_.shadingTriangles, model_.instances,
                                            perInstanceSettings_, instanceBounds_, request.width,
                                            request.height, threadPool_, gbuffer_);
        stats_.rasterMilliseconds =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - rasterStart).count();
    }

    // Filters read the accumulated Beauty, so they run after the accumulation loop, and each distinct one is evaluated once however many AOVs ask for it.
    filteredAovs_.clear();
    filtered_.clear();
    if (wantsFilter) {
        const auto filterStart = std::chrono::steady_clock::now();
        const engine::gfx::HdrImage& beauty = lastImage(AovId::Beauty);
        for (const AovId aov : request.aovs) {
            if (engine::debug::aovSource(aov) != AovSource::BeautyFilter ||
                std::find(filteredAovs_.begin(), filteredAovs_.end(), aov) != filteredAovs_.end()) {
                continue;
            }
            filteredAovs_.push_back(aov);
            switch (aov) {
                case AovId::Luminance: filtered_.push_back(engine::debug::luminanceAov(beauty, threadPool_)); break;
                case AovId::Sobel:     filtered_.push_back(engine::debug::sobelAov(beauty, threadPool_)); break;
                case AovId::Gabor:     filtered_.push_back(engine::debug::gaborAov(beauty, threadPool_)); break;
                default:               filtered_.push_back(engine::debug::hsvAov(beauty, threadPool_)); break;
            }
        }
        stats_.filterMilliseconds =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - filterStart).count();
    }

    return true;
}

}  // namespace engine::api
