#pragma once

#include <string_view>

namespace engine::debug {

// Single source of truth for every AOV the HUD can select and the path tracer can produce -- AppResources.aov stays a plain int (ImGui::Combo needs int&), cast via static_cast<AovId>(app.aov).
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

// Display name, index-parallel to AovId -- the single array HudOverlay's combo box binds to. Reordering must keep this parallel with AovId's declaration order; the static_assert below only catches a length mismatch, not a reorder.
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

// Which of the renderer's three producers computes each AOV. They are not interchangeable: the path tracer's 10 accumulated lanes (path_tracer.h's PathTraceResult), the rasterizer's 14 primary-hit lanes (rasterizer.h's RasterGBuffer), and 4 image-space filters over a finished Beauty (aov_filters.h).
// The single source of truth for producer selection. Previously this knowledge was restated in four places -- main.cpp's aovNeedsLightTransport and selectPathTracedImage, render_beauty's own lane table, and the README -- which is three chances for them to disagree about what produces what.
enum class AovSource { PathTraced, GBuffer, BeautyFilter };

[[nodiscard]] AovSource aovSource(AovId aov);

// Channels the AOV actually carries. A property of what it MEANS, not of how it is stored: HdrImage is always 4 floats/texel and scalar AOVs are broadcast to RGB so they can go straight to a display texture. A consumer reading the data rather than looking at it wants the one real channel of a depth map, not three copies of it.
[[nodiscard]] int aovChannels(AovId aov);

// True for AOVs needing light-transport data -- Beauty, the transport components, and the filters reading Beauty -- false for the 14 primary-hit-only AOVs the rasterizer covers. Selects which producer runs, and nothing else.
// Derived from aovSource rather than tabulated beside it, so the two cannot drift apart.
[[nodiscard]] inline bool aovNeedsLightTransport(AovId aov) {
    return aovSource(aov) != AovSource::GBuffer;
}

// Case- and separator-insensitive lookup against kAovNames, whose entries are HUD labels ("Bounce Count", "Indirect Specular"): "bounce-count", "bounce_count" and "bouncecount" all name the same AOV, rather than every consumer inventing a second vocabulary to keep in sync.
// AovId::Count when nothing matches -- the sentinel doubles as "unknown", since it is never itself a selectable AOV.
[[nodiscard]] AovId aovIdFromName(std::string_view name);

}  // namespace engine::debug
