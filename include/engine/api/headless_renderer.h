#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "engine/config/profile_config.h"
#include "engine/config/scene_config.h"
#include "engine/debug/aov.h"
#include "engine/debug/render_stats.h"
#include "engine/gfx/scalar_type.h"
#include "engine/gfx/hdr_image.h"
#include "engine/scene/camera.h"
#include "engine/scene/embree_accel.h"
#include "engine/scene/environment_map.h"
#include "engine/scene/gltf_loader.h"
#include "engine/scene/light.h"
#include "engine/scene/path_tracer.h"
#include "engine/scene/rasterizer.h"
#include "engine/scene/shading_scene.h"
#include "engine/scene/thread_pool.h"

namespace engine::api {

// A scene loaded once and rendered many times, with no window, no GL and no progressive driver -- the synchronous half of what PathTraceDriver does asynchronously for the viewer.
// Exists because the ~120 lines that turn a scene path into a traceable scene (profile/scene/material config, glTF load, film-back preset resolution, quad lights, per-instance material overrides, Embree build, environment map, light set, thread pool) were inlined in render_beauty's main() and reachable from nothing else. A foreign runtime cannot call a main().
// Owns everything renderPathTraced and renderRasterGBuffer need to be handed by reference, which is why it is neither copyable nor movable: LightSet stores a pointer to the EnvironmentMap beside it and a reference to the quad-light vector, and ThreadPool is non-movable by design. open() therefore hands back a unique_ptr rather than an optional -- the object's address is part of its own invariant.
class HeadlessRenderer {
public:
    struct Request {
        engine::scene::Camera camera;
        int width = 0;
        int height = 0;
        // Path-traced passes at one sample each, averaged. Ignored by a request whose AOVs all come from the rasterizer, which is not a sampled estimator and converges in one pass.
        int samples = 1;
        // Randomizes the sampler's Owen scramble. Held FIXED across the passes of one render (sampler.h) -- callers choose the seed, never the per-pass sample index, because supplying that pair wrongly silently degrades stratified Sobol to plain Monte Carlo.
        std::uint32_t scrambleSeed = 1;
        std::vector<engine::debug::AovId> aovs;
        // nullopt keeps the scene's own authored environment.lightEnabled; true/false override it, which is what lets one scene render both the lit and the classic unlit Cornell variant without a second scene.json.
        std::optional<bool> envLightEnabled;
    };

    // Per-pass wall clock and ray counts for the most recent render(), so a benchmark harness measures what the renderer actually did rather than re-deriving it. passMilliseconds is empty for a request that needed no light transport.
    struct RenderStats {
        // One entry per path-traced pass. Empty when the request needed no light transport, which is the normal case for a purely rasterizer-backed AOV -- not an error, and a consumer must not assume a pass happened.
        std::vector<double> passMilliseconds;
        // Wall clock of the single rasterizer scan-conversion and of the Beauty-filter evaluations. Zero when that producer did not run. Kept separate from passMilliseconds rather than appended to it: they measure different work, and averaging them together would describe neither.
        double rasterMilliseconds = 0.0;
        double filterMilliseconds = 0.0;
        engine::debug::RayCounts rays;
    };

    // nullopt-equivalent (nullptr) on any load failure, with the reason in `error`. assetRoot is the directory holding config/, scenes/, geometry/ and materials/; scenePath is relative to it.
    [[nodiscard]] static std::unique_ptr<HeadlessRenderer> open(const std::string& assetRoot,
                                                                 const std::string& scenePath,
                                                                 std::string& error);
    ~HeadlessRenderer();

    HeadlessRenderer(const HeadlessRenderer&) = delete;
    HeadlessRenderer& operator=(const HeadlessRenderer&) = delete;
    HeadlessRenderer(HeadlessRenderer&&) = delete;
    HeadlessRenderer& operator=(HeadlessRenderer&&) = delete;

    // profile.json's authored camera, the one render_beauty and the viewer both start from.
    [[nodiscard]] const engine::scene::Camera& defaultCamera() const { return defaultCamera_; }
    [[nodiscard]] int defaultWidth() const { return profile_.window.width; }
    [[nodiscard]] int defaultHeight() const { return profile_.window.height; }

    // Blocking. outputs is parallel to request.aovs, and outputs[i] must hold width * height * aovChannels(aovs[i]) floats -- caller-allocated so no buffer ownership crosses the boundary (the C ABI hands numpy's own memory straight through).
    // Each producer runs AT MOST ONCE regardless of how many of its AOVs are asked for: the 10 path-traced lanes share one sample set and one reconstruction filter, the 13 rasterizer lanes share one scan-conversion, and the 4 filters share the one accumulated Beauty. Asking for beauty+depth+normal+sobel is one accumulation, one rasterizer pass and one filter -- not four renders.
    [[nodiscard]] bool render(const Request& request, std::span<float* const> outputs,
                               std::string& error);

    // Same render with no packing step, for a C++ caller that reads the results through lastImage() and wants the 4-channel HdrImage rather than tightly packed channels.
    [[nodiscard]] bool render(const Request& request, std::string& error);

    // The full 4-channel buffer behind one AOV of the most recent render(), for a consumer that wants HdrImage itself -- an EXR write or a display encode -- rather than the packed channels render() hands back. Valid until the next render(); the AOV must have been part of that request.
    [[nodiscard]] const engine::gfx::HdrImage& lastImage(engine::debug::AovId aov) const;
    [[nodiscard]] const RenderStats& lastStats() const { return stats_; }

    // Resolved scene state a caller may need to REPORT rather than to render with -- a benchmark record naming the settings a timing was measured under.
    [[nodiscard]] const engine::scene::PathTraceSettings& baseSettings() const { return baseSettings_; }
    [[nodiscard]] engine::gfx::ScalarType textureType() const { return profile_.render.textureType; }
    [[nodiscard]] bool defaultEnvLightEnabled() const { return defaultEnvLightEnabled_; }

private:
    HeadlessRenderer(engine::config::ProfileConfig profile,
                     engine::scene::LoadedModel model, std::vector<engine::scene::QuadLight> quadLights,
                     std::vector<int> instanceLightIndex,
                     std::vector<engine::scene::PathTraceSettings> perInstanceSettings,
                     engine::scene::PathTraceSettings baseSettings, engine::scene::EmbreeAccel accel,
                     engine::gfx::ImageTexture environmentImage, bool envLightEnabled,
                     engine::scene::Camera defaultCamera);

    // Sizes the reused path-traced and rasterizer buffers to this request, reallocating only on a resolution change.
    void resizeBuffers(int width, int height);

    engine::config::ProfileConfig profile_;
    engine::scene::LoadedModel model_;
    std::vector<int> instanceLightIndex_;
    std::vector<engine::scene::PathTraceSettings> perInstanceSettings_;
    engine::scene::PathTraceSettings baseSettings_;
    std::vector<engine::scene::AabbBounds> instanceBounds_;
    engine::scene::EmbreeAccel accel_;
    // Declaration order is load-bearing: lights_ captures &environmentMap_ and a reference to quadLights_, so both must be constructed before it.
    engine::scene::EnvironmentMap environmentMap_;
    std::vector<engine::scene::QuadLight> quadLights_;
    // Both light sets are built at load and chosen between per request: LightSet only stores a pointer and a reference, so holding the environment-off variant costs nothing and keeps "constructed once" true.
    engine::scene::LightSet lights_;
    engine::scene::LightSet lightsEnvOff_;
    bool defaultEnvLightEnabled_ = true;
    engine::scene::Camera defaultCamera_;
    engine::scene::ThreadPool threadPool_;

    // Reused across render() calls, reallocated only when the resolution changes.
    engine::scene::PathTraceResult pathTraced_;
    engine::scene::RasterGBuffer gbuffer_;
    // One running sum per path-traced lane this request needs, parallel to accumulatedAovs_.
    std::vector<engine::debug::AovId> accumulatedAovs_;
    std::vector<engine::gfx::HdrImage> accumulators_;
    // One evaluated filter per distinct BeautyFilter AOV this request needs, parallel to filteredAovs_. Held rather than computed into a temporary so lastImage() can hand one back, and so two AOVs asking for the same filter evaluate it once.
    std::vector<engine::debug::AovId> filteredAovs_;
    std::vector<engine::gfx::HdrImage> filtered_;
    RenderStats stats_;
    int bufferWidth_ = 0;
    int bufferHeight_ = 0;
};

}  // namespace engine::api
