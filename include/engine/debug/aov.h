#pragma once

namespace engine::debug {

// Single source of truth for every AOV the HUD can select, replacing the old, separately
// maintained kAovNames (rasterizer, pbr.frag-driven) and kPathAovNames (path tracer) arrays that
// were kept in sync only by comments. Values 0-17 are numerically unchanged from the pre-unification
// kAovNames so pbr.frag's uAov branches and main.cpp's aov==0/5/6 literals (drawSky's sky gate,
// presentFrame's Sobel/Gabor gate) don't need renumbering -- AppResources.aov stays a plain int
// (ImGui::Combo needs int&), cast via static_cast<AovId>(app.aov).
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
    // Path-tracer-only diagnostics (previously hud_overlay.cpp's separate kPathAovNames).
    IOR,
    BounceCount,
    // Light-transport component breakdown, replacing the old single "IBL" entry.
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

// Maps the unified AovId to the rasterizer fallback's legacy pbr.frag uAov index (0-18, where 18 was
// "IBL", now orphaned/unused by this mapping but not deleted from pbr.frag this pass). AOVs pbr.frag
// has no branch for (IOR, BounceCount, and the five transport-component AOVs) fall back to Beauty
// rather than misleadingly reusing pbr.frag's old IBL branch under a different AOV's name.
[[nodiscard]] constexpr int toRasterAovIndex(AovId aov) {
    const int raw = static_cast<int>(aov);
    return raw <= 17 ? raw : 0;
}

}  // namespace engine::debug
