#pragma once

namespace engine::debug {

// Per-frame scene/viewport counts for the HUD's Scene section. Populated
// by main.cpp's existing per-instance draw loop, which already computes
// the frustum-culling decision this just tallies.
struct SceneStats {
    int objectCount;
    int instancesDrawn;
    int instancesCulled;
    long long trianglesTotal;
    long long trianglesDrawn;
    long long pointsTotal;
    int viewportWidth;
    int viewportHeight;
};

}  // namespace engine::debug
