#pragma once

#include <optional>
#include <string>

namespace engine::config {

// Tunable shading constants that shape how a material's textures get turned into BSDF input -- externalized so these can be edited without recompiling; see assets/config/material.json for the checked-in defaults.
struct MaterialConfig {
    float bumpStrength;   // scales the bump texture's raw per-texel height difference; see path_tracer.cpp's buildShadingFrame
    float roughnessMin;   // floor applied to the roughness texture sample, avoids a near-zero-roughness GGX singularity
    float roughnessMax;   // ceiling applied to the roughness texture sample
};

// Reads and parses path. Returns nullopt and logs to stderr if the file is missing, unreadable, or any required field can't be found/parsed — this is user-editable input, not an internal invariant, so failure is expected to happen and is surfaced rather than defaulted around.
[[nodiscard]] std::optional<MaterialConfig> loadMaterialConfig(const std::string& path);

}  // namespace engine::config
