#pragma once

namespace engine::debug {

// Single source of truth for every AOV the HUD can select and the path tracer can produce.
// AppResources.aov stays a plain int (ImGui::Combo needs int&), cast via static_cast<AovId>(app.aov).
enum class AovId : int {
    Beauty = 0,
    Alpha,
    Depth,
    HSV,
    Luminance,
    Sobel,
    Gabor,
    WorldPos,
    UV,
    Normal,
    GeomNormal,
    Albedo,
    Metallic,
    Roughness,
    Tangent,
    ObjectID,
    AO,
    Fresnel,
    IOR,
    BounceCount,
    // Light-transport component breakdown.
    DirectDiffuse,
    IndirectDiffuse,
    DirectSpecular,
    IndirectSpecular,
    Refraction,
    Count  // sentinel, == array size, not itself a selectable value
};

// Display name, index-parallel to AovId -- the single array HudOverlay's combo box binds to.
// Reordering must keep this parallel with AovId's declaration order; the static_assert below only
// catches a length mismatch, not a reorder.
inline constexpr const char* kAovNames[] = {
    "Beauty",       "Alpha",          "Depth",           "HSV",
    "Luminance",    "Sobel",          "Gabor",           "WorldPos",
    "UV",           "Normal",         "GeomNormal",      "Albedo",
    "Metallic",     "Roughness",      "Tangent",         "ObjectID",
    "AO",           "Fresnel",        "IOR",             "Bounce Count",
    "Direct Diffuse", "Indirect Diffuse", "Direct Specular",
    "Indirect Specular", "Refraction",
};
static_assert(sizeof(kAovNames) / sizeof(kAovNames[0]) == static_cast<int>(AovId::Count),
              "kAovNames must stay index-parallel with AovId");

}  // namespace engine::debug
