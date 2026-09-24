#pragma once

namespace pathtracer::debug {

// Per-frame scene/viewport counts for the HUD's Scene section.
struct SceneStats {
    int objectCount;
    long long trianglesTotal;
    long long pointsTotal;
    int viewportWidth;
    int viewportHeight;
};

}  // namespace pathtracer::debug
