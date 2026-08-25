#pragma once

#include <optional>
#include <string>
#include <string_view>

#include <glm/glm.hpp>

// Minimal, purpose-built JSON field extraction for this engine's small, known set of flat-ish config fields (scene.json/profile.json) — not a general JSON parser, same "just enough" philosophy as gltf_loader.cpp's extrasTextureIndex helper. Handles top-level and one-level-nested numbers/strings/vec3 arrays; no arbitrary nesting depth.
namespace engine::config::json {

[[nodiscard]] std::optional<std::string> readFile(const std::string& path);

// Locates "key": <number> anywhere in text and parses the number.
[[nodiscard]] std::optional<double> findNumber(std::string_view text, std::string_view key);

// Locates "key": "value" anywhere in text.
[[nodiscard]] std::optional<std::string> findString(std::string_view text, std::string_view key);

// Locates "key": [a, b, c] anywhere in text.
[[nodiscard]] std::optional<glm::vec3> findVec3(std::string_view text, std::string_view key);

// Locates "key": { ... } and returns the substring between (and excluding) the outer braces, for scoping a nested lookup (e.g. filmBack/light) so a nested field name can't collide with a top-level one of the same name. Returns an empty view if not found.
[[nodiscard]] std::string_view findObjectBody(std::string_view text, std::string_view key);

}  // namespace engine::config::json
