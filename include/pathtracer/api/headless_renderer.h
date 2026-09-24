#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "pathtracer/config/profile_config.h"
#include "pathtracer/config/scene_config.h"
#include "pathtracer/debug/aov.h"
#include "pathtracer/debug/render_stats.h"
#include "pathtracer/gfx/scalar_type.h"
#include "pathtracer/gfx/hdr_image.h"
#include "pathtracer/scene/camera.h"
#include "pathtracer/scene/embree_accel.h"
#include "pathtracer/scene/environment_map.h"
#include "pathtracer/scene/gltf_loader.h"
#include "pathtracer/scene/light.h"
#include "pathtracer/scene/path_tracer.h"
#include "pathtracer/scene/rasterizer.h"
#include "pathtracer/scene/shading_scene.h"
#include "pathtracer/scene/thread_pool.h"

namespace pathtracer::api {

// A scene loaded once and rendered many times, with no window, no GL and no progressive driver -- the synchronous half
// of what PathTraceDriver does asynchronously for the viewer. Neither copyable nor movable: lights_ holds a pointer to
// environmentMap_ and a reference to quadLights_, and ThreadPool owns running threads.
class HeadlessRenderer {
public:
    struct Request {
        pathtracer::scene::Camera camera;
        int width = 0;
        int height = 0;
        // Path-traced passes at one sample each, averaged. Ignored when every requested AOV comes from the rasterizer,
        // which is not a sampled estimator and converges in one pass.
        int samples = 1;
        // Randomizes the sampler's Owen scramble, held fixed across the passes of one render (sampler.h). Callers choose
        // the seed, never the per-pass sample index: supplying that pair wrongly degrades stratified sampling silently.
        std::uint32_t scrambleSeed = 1;
        std::vector<pathtracer::debug::AovId> aovs;
        // nullopt keeps the scene's authored environment.lightEnabled; true/false override it, which is what renders both
        // the lit and the classic unlit Cornell from one scene.json.
        std::optional<bool> envLightEnabled;
    };

    // Per-pass wall clock and ray counts for the most recent render(), so a benchmark measures what the renderer did
    // rather than re-deriving it.
    struct RenderStats {
        // One entry per path-traced pass. Empty when the request needed no light transport, the normal case for a
        // rasterizer-backed AOV -- not an error, and a consumer must not assume a pass happened.
        std::vector<double> passMilliseconds;
        // Wall clock of the one scan-conversion and of the Beauty-filter evaluations, zero when that producer did not
        // run. Separate from passMilliseconds: they measure different work, so averaging them together means nothing.
        double rasterMilliseconds = 0.0;
        double filterMilliseconds = 0.0;
        pathtracer::debug::RayCounts rays;
    };

    // nullptr on any load failure, with the reason in `error`. assetRoot holds config/, scenes/, geometry/ and
    // materials/; scenePath is relative to it.
    [[nodiscard]] static std::unique_ptr<HeadlessRenderer> open(const std::string& assetRoot,
                                                                 const std::string& scenePath,
                                                                 std::string& error);
    ~HeadlessRenderer();

    HeadlessRenderer(const HeadlessRenderer&) = delete;
    HeadlessRenderer& operator=(const HeadlessRenderer&) = delete;
    HeadlessRenderer(HeadlessRenderer&&) = delete;
    HeadlessRenderer& operator=(HeadlessRenderer&&) = delete;

    // profile.json's authored camera, the one render_beauty and the viewer both start from.
    [[nodiscard]] const pathtracer::scene::Camera& defaultCamera() const { return defaultCamera_; }
    [[nodiscard]] int defaultWidth() const { return profile_.window.width; }
    [[nodiscard]] int defaultHeight() const { return profile_.window.height; }

    // Blocking. outputs is parallel to request.aovs, and outputs[i] must hold width * height * aovChannels(aovs[i])
    // floats -- caller-allocated, so no buffer ownership crosses the boundary (the C ABI passes numpy memory straight
    // through). Each producer runs at most once however many of its AOVs are asked for.
    [[nodiscard]] bool render(const Request& request, std::span<float* const> outputs,
                               std::string& error);

    // Same render with no packing step, for a C++ caller reading results through lastImage() as 4-channel HdrImage.
    [[nodiscard]] bool render(const Request& request, std::string& error);

    // The full 4-channel buffer behind one AOV of the most recent render(), for a consumer wanting HdrImage itself --
    // an EXR write or a display encode. Valid until the next render(), and the AOV must have been requested.
    [[nodiscard]] const pathtracer::gfx::HdrImage& lastImage(pathtracer::debug::AovId aov) const;
    [[nodiscard]] const RenderStats& lastStats() const { return stats_; }

    // Resolved scene state a caller may need to report rather than render with: a benchmark naming its settings.
    [[nodiscard]] const pathtracer::scene::PathTraceSettings& baseSettings() const { return baseSettings_; }
    [[nodiscard]] pathtracer::gfx::ScalarType textureType() const { return profile_.render.textureType; }
    [[nodiscard]] bool defaultEnvLightEnabled() const { return defaultEnvLightEnabled_; }

private:
    HeadlessRenderer(pathtracer::config::ProfileConfig profile,
                     pathtracer::scene::LoadedModel model, std::vector<pathtracer::scene::QuadLight> quadLights,
                     std::vector<int> instanceLightIndex,
                     std::vector<pathtracer::scene::PathTraceSettings> perInstanceSettings,
                     pathtracer::scene::PathTraceSettings baseSettings, pathtracer::scene::EmbreeAccel accel,
                     pathtracer::gfx::ImageTexture environmentImage, bool envLightEnabled,
                     const pathtracer::scene::Camera& defaultCamera);

    // Sizes the reused path-traced and rasterizer buffers to this request, reallocating only on a resolution change.
    void resizeBuffers(int width, int height);

    pathtracer::config::ProfileConfig profile_;
    pathtracer::scene::LoadedModel model_;
    std::vector<int> instanceLightIndex_;
    std::vector<pathtracer::scene::PathTraceSettings> perInstanceSettings_;
    pathtracer::scene::PathTraceSettings baseSettings_;
    std::vector<pathtracer::scene::AabbBounds> instanceBounds_;
    pathtracer::scene::EmbreeAccel accel_;
    // Declaration order is load-bearing: lights_ captures &environmentMap_ and a reference to quadLights_.
    pathtracer::scene::EnvironmentMap environmentMap_;
    std::vector<pathtracer::scene::QuadLight> quadLights_;
    // Both light sets are built at load and chosen per request: LightSet stores only a pointer and a reference, so the
    // environment-off variant costs nothing and keeps "constructed once" true.
    pathtracer::scene::LightSet lights_;
    pathtracer::scene::LightSet lightsEnvOff_;
    bool defaultEnvLightEnabled_ = true;
    pathtracer::scene::Camera defaultCamera_;
    pathtracer::scene::ThreadPool threadPool_;

    // Reused across render() calls, reallocated only when the resolution changes.
    pathtracer::scene::PathTraceResult pathTraced_;
    pathtracer::scene::RasterGBuffer gbuffer_;
    // One running sum per path-traced lane this request needs, parallel to accumulatedAovs_.
    std::vector<pathtracer::debug::AovId> accumulatedAovs_;
    std::vector<pathtracer::gfx::HdrImage> accumulators_;
    // One evaluated filter per distinct BeautyFilter AOV this request needs, parallel to filteredAovs_. Held rather
    // than computed into a temporary so lastImage() can return one, and so two AOVs sharing a filter evaluate it once.
    std::vector<pathtracer::debug::AovId> filteredAovs_;
    std::vector<pathtracer::gfx::HdrImage> filtered_;
    RenderStats stats_;
    int bufferWidth_ = 0;
    int bufferHeight_ = 0;
};

}  // namespace pathtracer::api
