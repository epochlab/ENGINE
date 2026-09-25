#pragma once

namespace pathtracer::debug {

// Per-frame scene counts and the authored resolution, for the HUD's Resolution and Scene sections.
struct SceneStats {
    int objectCount;
    long long trianglesTotal;
    long long pointsTotal;
    int imageWidth;
    int imageHeight;
};

}  // namespace pathtracer::debug
