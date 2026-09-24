#pragma once

#include <string_view>

namespace pathtracer::debug {

// Single source of truth for every AOV the HUD can select and the path tracer can produce. AppResources.aov stays a
// plain int because ImGui::Combo needs int&, and is cast to AovId at every use.
// Grouped by category (docs/aovs.md): Utility, Material, Transport, Lighting.
enum class AovId : int {
    // Utility.
    Beauty = 0,
    Wireframe,  // combined AOV: white mesh-edge lines + one false-coloured bounding box per instance (rasterizer.h)
    Alpha,
    Depth,
    Lookahead,  // Depth remapped through profile.json's lookaheadDistance: 1 at the camera plane, 0 at that horizon and beyond
    HSV,
    Luminance,
    Sobel,
    Gabor,
    WorldPos,
    UV,
    // Material.
    Normal,
    GeomNormal,
    Albedo,
    Metallic,
    Roughness,
    Tangent,
    ObjectID,
    AO,
    // Transport.
    Fresnel,
    IOR,
    BounceCount,
    // Lighting.
    DirectDiffuse,
    IndirectDiffuse,
    DirectSpecular,
    IndirectSpecular,
    Refraction,
    Shadow,
    Count  // sentinel, == array size, not itself a selectable value
};

// Display name, index-parallel to AovId -- the array HudOverlay's combo box binds to. Reordering must keep the two
// parallel; the static_assert below is the gate on that.
inline constexpr const char* kAovNames[] = {
    "Beauty",       "Wireframe",      "Alpha",           "Depth",
    "Lookahead",    "HSV",            "Luminance",       "Sobel",
    "Gabor",        "WorldPos",       "UV",
    "Normal",       "GeomNormal",     "Albedo",          "Metallic",
    "Roughness",    "Tangent",        "ObjectID",        "AO",
    "Fresnel",      "IOR",            "Bounce Count",
    "Direct Diffuse", "Indirect Diffuse", "Direct Specular",
    "Indirect Specular", "Refraction", "Shadow",
};
static_assert(sizeof(kAovNames) / sizeof(kAovNames[0]) == static_cast<int>(AovId::Count),
              "kAovNames must stay index-parallel with AovId");

// Which of the renderer's three producers computes each AOV. They are not interchangeable: 10 accumulated path-traced
// lanes, 14 primary-hit rasterizer lanes, and 4 filters reading Beauty.
// The single source of truth for producer selection, previously restated in four places that could disagree.
enum class AovSource { PathTraced, GBuffer, BeautyFilter };

[[nodiscard]] AovSource aovSource(AovId aov);

// Channels the AOV actually carries: a property of what it means, not how it is stored. HdrImage is always 4
// floats/texel and scalar AOVs are broadcast to RGB, so this is what a packed consumer must allocate for.
[[nodiscard]] int aovChannels(AovId aov);

// True for AOVs needing light transport -- Beauty, the transport components and the filters over Beauty -- false for
// the 14 primary-hit-only AOVs the rasterizer covers.
// Derived from aovSource rather than tabulated beside it, so the two cannot drift apart.
[[nodiscard]] inline bool aovNeedsLightTransport(AovId aov) {
    return aovSource(aov) != AovSource::GBuffer;
}

// Case- and separator-insensitive lookup against kAovNames, whose entries are HUD labels ("Bounce Count"), so
// "bounce-count", "bounce_count" and "bouncecount" all name the same AOV.
// AovId::Count when nothing matches -- the sentinel doubles as "unknown", since it is never itself a selectable AOV.
[[nodiscard]] AovId aovIdFromName(std::string_view name);

}  // namespace pathtracer::debug
