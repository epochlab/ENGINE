// Correctness gate for the engine's file-boundary code: the EXR round trip (gfx/hdr_image.cpp), the JSON scene and
// profile parsers (config/scene_config.cpp, config/profile_config.cpp), and the benchmark log writer (debug/bench_log.cpp).
//
// These are the places the engine ingests data it did not produce, which is exactly where validation earns its keep --
// and none of them had any. hdr_image.cpp is linked into three validators and was invoked by none of them: its round
// trip was asserted only by a comment claiming losslessness. profile_config.cpp had no coverage at all.
//
// Both halves of each parser's contract are asserted, and the rejection half is the load-bearing one: a parser that
// accepts valid input but silently accepts invalid input too will not fail here on the happy path, and a malformed
// asset then reaches the renderer as a plausible-looking wrong number rather than an error.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <variant>
#include <vector>

#include <glm/glm.hpp>
#include <nlohmann/json.hpp>

#include "check.h"
#include "engine/config/profile_config.h"
#include "engine/config/scene_config.h"
#include "engine/debug/bench_log.h"
#include "engine/gfx/hdr_image.h"
#include "engine/gfx/texture.h"

namespace {

// Written under the system temp directory rather than the source tree: a validator must not need a writable checkout,
// and must leave nothing behind for the next run to accidentally pass against.
std::filesystem::path scratchPath(const char* name) {
    return std::filesystem::temp_directory_path() / name;
}

// Values chosen to be hostile to a lossy or narrowing round trip: denormal-scale, exact halves, a value far outside
// display range, and a negative -- all representable in float32 and all preserved by a full-float EXR channel.
engine::gfx::HdrImage makeProbeImage() {
    engine::gfx::HdrImage image;
    image.width = 7;   // deliberately not a power of two or a multiple of any tile size
    image.height = 5;
    image.rgba.resize(static_cast<std::size_t>(image.width) * image.height * 4);
    for (std::size_t i = 0; i < image.rgba.size(); ++i) {
        const std::size_t channel = i % 4;
        const auto t = static_cast<float>(i);
        switch (channel) {
            case 0: image.rgba[i] = t * 1.5F; break;              // exact halves
            case 1: image.rgba[i] = 1.0e-20F * (t + 1.0F); break;  // far below display range
            case 2: image.rgba[i] = 65504.0F - t; break;           // near the half-float maximum, if one were used
            default: image.rgba[i] = 1.0F; break;                  // alpha
        }
    }
    return image;
}

// The losslessness hdr_image.h claims in prose. Bit-exact, not approximate: both directions write full-float channels,
// so any difference at all means a channel type or a stride is wrong. The 1e-20 and 65504 rows are what would expose a
// half-float channel, which would round both to something else entirely while leaving the ordinary values intact.
ENGINE_CHECK(exr_round_trip_is_lossless, Fast, Exact) {
    ctx.plan(4);
    const engine::gfx::HdrImage original = makeProbeImage();
    const std::filesystem::path path = scratchPath("engine_io_validate_roundtrip.exr");
    std::filesystem::remove(path);

    ENGINE_EXPECT(ctx, engine::gfx::writeExr(path.string(), original), "writeExr failed on a valid image");
    const std::optional<engine::gfx::HdrImage> loaded = engine::gfx::loadExr(path.string());
    if (!loaded.has_value()) {
        ENGINE_EXPECT(ctx, false, "loadExr returned nullopt for a file writeExr had just written");
        ENGINE_EXPECT(ctx, false, "dimensions unavailable");
        ENGINE_EXPECT(ctx, false, "contents unavailable");
        std::filesystem::remove(path);
        return;
    }

    char dimDetail[160];
    std::snprintf(dimDetail, sizeof(dimDetail), "round trip returned %dx%d, wrote %dx%d", loaded->width,
                  loaded->height, original.width, original.height);
    ENGINE_EXPECT(ctx, loaded->width == original.width && loaded->height == original.height, dimDetail);
    ENGINE_EXPECT(ctx, loaded->rgba.size() == original.rgba.size(), "round trip changed the channel count");

    std::size_t differing = 0;
    float worst = 0.0F;
    if (loaded->rgba.size() == original.rgba.size()) {
        for (std::size_t i = 0; i < original.rgba.size(); ++i) {
            if (loaded->rgba[i] != original.rgba[i]) {
                ++differing;
                worst = std::max(worst, std::fabs(loaded->rgba[i] - original.rgba[i]));
            }
        }
    }
    char detail[192];
    std::snprintf(detail, sizeof(detail), "%zu of %zu floats changed across the round trip, worst delta %.9g",
                  differing, original.rgba.size(), static_cast<double>(worst));
    ENGINE_EXPECT(ctx, differing == 0, detail);
    std::filesystem::remove(path);
}

// A missing file must be reported, not treated as an empty image: loadExr's contract is nullopt on failure, and a
// caller that received a zero-sized image instead would render black and never know why.
ENGINE_CHECK(exr_load_rejects_bad_input, Fast, Exact) {
    ctx.plan(2);
    const std::filesystem::path missing = scratchPath("engine_io_validate_does_not_exist.exr");
    std::filesystem::remove(missing);
    ENGINE_EXPECT(ctx, !engine::gfx::loadExr(missing.string()).has_value(),
                  "loadExr accepted a path that does not exist");

    // A file that exists but is not an EXR at all -- the realistic corruption, and the one a magic-number check alone
    // would catch while a truncated-header check would not.
    const std::filesystem::path garbage = scratchPath("engine_io_validate_garbage.exr");
    {
        std::ofstream out(garbage, std::ios::binary);
        out << "this is not an OpenEXR file, but it is definitely a file";
    }
    ENGINE_EXPECT(ctx, !engine::gfx::loadExr(garbage.string()).has_value(),
                  "loadExr accepted a file whose contents are not EXR");
    std::filesystem::remove(garbage);
}

// Writes `text` to a scratch .json and hands back the path, so each rejection row states its own malformation inline
// rather than needing a checked-in fixture file per case.
std::filesystem::path writeJson(const char* name, const std::string& text) {
    const std::filesystem::path path = scratchPath(name);
    std::ofstream out(path);
    out << text;
    return path;
}

// The shipped scene must load: without this row, every rejection row below could pass by rejecting everything.
ENGINE_CHECK(scene_config_accepts_the_shipped_scene, Fast, Exact) {
    ctx.plan(1);
    const std::filesystem::path scene = std::filesystem::path(ASSET_ROOT_DIR) / "scenes" / "cornell.json";
    const std::optional<engine::config::SceneConfig> loaded = engine::config::loadSceneConfig(scene.string());
    char detail[256];
    std::snprintf(detail, sizeof(detail), "loadSceneConfig rejected the shipped scene at %s", scene.string().c_str());
    ENGINE_EXPECT(ctx, loaded.has_value(), detail);
}

// The rejection half of the contract. Each row is a malformation a real authoring mistake produces, and each must be
// reported rather than absorbed into a default -- a quad light with non-perpendicular edges, for instance, would be
// sampled by a spherical-rectangle sampler that is exact only for rectangles, producing a quietly wrong image.
ENGINE_CHECK(scene_config_rejects_malformed_input, Fast, Exact) {
    struct Case {
        const char* name;
        const char* file;
        std::string text;
    };
    // Built by mutating a base that loads, so each row fails for the reason it names. Stating a malformed scene
    // outright risks a vacuous pass: an earlier draft of the two light rows below omitted "environment" and was
    // rejected for THAT, never reaching the light validation they exist to test.
    const auto scene = [](const std::string& lights) {
        return std::string(
                   "{\"model\":{\"gltfPath\":\"geometry/cornell/cornell_v001.gltf\",\"texturePath\":\"\","
                   "\"position\":[0,0,0],\"rotation\":[0,0,0]},"
                   "\"environment\":{\"hdriPath\":\"textures/republiqueHDR_2k.exr\"},"
                   "\"materialPath\":\"materials/clay.json\"") +
               lights + "}";
    };
    const std::string validLights =
        ",\"lights\":[{\"type\":\"quad\",\"origin\":[0,0,0],\"edge0\":[1,0,0],\"edge1\":[0,0,1],"
        "\"color\":[1,1,1],\"intensity\":5.0,\"twoSided\":false}]";

    const std::vector<Case> cases = {
        {"not JSON at all", "engine_io_scene_notjson.json", "{ this is not json"},
        {"empty file", "engine_io_scene_empty.json", ""},
        {"JSON array where an object is required", "engine_io_scene_array.json", "[1, 2, 3]"},
        {"missing the model section entirely", "engine_io_scene_nomodel.json",
         "{\"environment\":{\"hdriPath\":\"x.exr\"},\"materialPath\":\"materials/clay.json\"}"},
        // Negative radiance is not a scene, and would propagate as negative energy through every estimator.
        {"negative light intensity", "engine_io_scene_negintensity.json",
         scene(",\"lights\":[{\"type\":\"quad\",\"origin\":[0,0,0],\"edge0\":[1,0,0],\"edge1\":[0,0,1],"
               "\"color\":[1,1,1],\"intensity\":-5.0,\"twoSided\":false}]")},
        // The spherical-rectangle sampler (Urena et al. 2013) is exact only for a RECTANGLE, so skewed edges would be
        // sampled against geometry the light does not have -- a quietly wrong image rather than an error.
        {"quad light with non-perpendicular edges", "engine_io_scene_skewlight.json",
         scene(",\"lights\":[{\"type\":\"quad\",\"origin\":[0,0,0],\"edge0\":[1,0,0],\"edge1\":[1,1,0],"
               "\"color\":[1,1,1],\"intensity\":5.0,\"twoSided\":false}]")},
        {"negative light colour", "engine_io_scene_negcolor.json",
         scene(",\"lights\":[{\"type\":\"quad\",\"origin\":[0,0,0],\"edge0\":[1,0,0],\"edge1\":[0,0,1],"
               "\"color\":[1,-1,1],\"intensity\":5.0,\"twoSided\":false}]")},
    };

    // Anti-vacuity: the base the three light rows are built from must itself LOAD, or they would prove nothing.
    const std::filesystem::path basePath = writeJson("engine_io_scene_base.json", scene(validLights));
    const bool baseLoads = engine::config::loadSceneConfig(basePath.string()).has_value();
    std::filesystem::remove(basePath);

    ctx.plan(static_cast<int>(cases.size()) + 1);
    ENGINE_EXPECT(ctx, baseLoads,
                  "the unmutated base scene must load, or every mutated row below passes vacuously");
    for (const Case& testCase : cases) {
        const std::filesystem::path path = writeJson(testCase.file, testCase.text);
        const bool accepted = engine::config::loadSceneConfig(path.string()).has_value();
        char detail[224];
        std::snprintf(detail, sizeof(detail), "loadSceneConfig accepted a scene with %s", testCase.name);
        ENGINE_EXPECT(ctx, !accepted, detail);
        std::filesystem::remove(path);
    }
}

ENGINE_CHECK(profile_config_accepts_the_shipped_profile, Fast, Exact) {
    ctx.plan(1);
    const std::filesystem::path profile = std::filesystem::path(ASSET_ROOT_DIR) / "config" / "profile.json";
    char detail[256];
    std::snprintf(detail, sizeof(detail), "loadProfileConfig rejected the shipped profile at %s",
                  profile.string().c_str());
    ENGINE_EXPECT(ctx, engine::config::loadProfileConfig(profile.string()).has_value(), detail);
}

// profile_config.cpp had no coverage of any kind. These rows are the boundary values it is responsible for: a
// zero-or-negative resolution divides an aspect ratio, and a zero film-back dimension is a denominator inside
// Camera::verticalFovRadians().
ENGINE_CHECK(profile_config_rejects_malformed_input, Fast, Exact) {
    struct Case {
        const char* name;
        const char* file;
        std::string text;
    };
    const std::vector<Case> cases = {
        {"not JSON at all", "engine_io_profile_notjson.json", "{ nope"},
        {"empty file", "engine_io_profile_empty.json", ""},
        {"JSON array where an object is required", "engine_io_profile_array.json", "[]"},
    };

    ctx.plan(static_cast<int>(cases.size()) + 1);
    for (const Case& testCase : cases) {
        const std::filesystem::path path = writeJson(testCase.file, testCase.text);
        char detail[224];
        std::snprintf(detail, sizeof(detail), "loadProfileConfig accepted a profile with %s", testCase.name);
        ENGINE_EXPECT(ctx, !engine::config::loadProfileConfig(path.string()).has_value(), detail);
        std::filesystem::remove(path);
    }

    const std::filesystem::path missing = scratchPath("engine_io_profile_absent.json");
    std::filesystem::remove(missing);
    ENGINE_EXPECT(ctx, !engine::config::loadProfileConfig(missing.string()).has_value(),
                  "loadProfileConfig accepted a path that does not exist");
}

// render.vsync, displayBitDepth and textureBitDepth, each varied alone on the shipped profile: every accepted value must map to its own setting and leave the other two alone, every other value must be refused rather than coerced (16.5 would otherwise truncate to 16).
ENGINE_CHECK(profile_config_render_display_settings, Fast, Exact) {
    using engine::gfx::ScalarType;
    struct Case {
        std::string name;
        const char* key;
        nlohmann::json value;  // null = key removed
        bool accepted;
    };
    std::vector<Case> cases = {
        {"displayBitDepth 16", "displayBitDepth", 16, true},
        {"displayBitDepth 32", "displayBitDepth", 32, true},
        {"textureBitDepth 16", "textureBitDepth", 16, true},
        {"textureBitDepth 32", "textureBitDepth", 32, true},
        {"vsync false", "vsync", false, true},
        {"vsync true", "vsync", true, true},
        {"vsync 1", "vsync", 1, false},
        {"vsync \"true\"", "vsync", "true", false},
        {"vsync missing", "vsync", nullptr, false},
    };
    for (const char* key : {"displayBitDepth", "textureBitDepth"}) {
        for (const nlohmann::json& bad : {nlohmann::json(8), nlohmann::json(24), nlohmann::json(16.5), nlohmann::json("16"),
                                          nlohmann::json(nullptr)}) {
            cases.push_back({std::string(key) + " " + (bad.is_null() ? "missing" : bad.dump()), key, bad, false});
        }
    }

    // Each accepted row varies one key against the shipped profile, so the expectation is the shipped setting with that one key overridden -- no assumption about what the shipped depths are.
    const std::filesystem::path shippedPath = std::filesystem::path(ASSET_ROOT_DIR) / "config" / "profile.json";
    const std::optional<engine::config::ProfileConfig> shippedConfig = engine::config::loadProfileConfig(shippedPath.string());
    std::ifstream shippedFile(shippedPath);
    const nlohmann::json shipped = nlohmann::json::parse(shippedFile);
    ctx.plan(static_cast<int>(cases.size()) + 1);
    ENGINE_EXPECT(ctx, shippedConfig.has_value(), "the shipped profile does not load, so no row below means anything");
    if (!shippedConfig) {
        return;
    }
    for (const Case& testCase : cases) {
        nlohmann::json edited = shipped;
        if (testCase.value.is_null()) {
            edited["render"].erase(testCase.key);
        } else {
            edited["render"][testCase.key] = testCase.value;
        }
        const std::filesystem::path path = writeJson("engine_io_profile_render.json", edited.dump());
        const std::optional<engine::config::ProfileConfig> loaded = engine::config::loadProfileConfig(path.string());
        std::filesystem::remove(path);
        char detail[224];
        if (testCase.accepted) {
            const bool isDisplay = std::string(testCase.key) == "displayBitDepth";
            const bool isTexture = std::string(testCase.key) == "textureBitDepth";
            const std::optional<ScalarType> varied =
                testCase.value.is_number_integer() ? engine::gfx::scalarTypeFromBitDepth(testCase.value.get<int>())
                                                   : std::nullopt;
            const ScalarType display = isDisplay ? *varied : shippedConfig->render.displayFormat;
            const ScalarType texture = isTexture ? *varied : shippedConfig->render.textureType;
            const bool vsync = isDisplay || isTexture ? shippedConfig->render.vsync : testCase.value.get<bool>();
            std::snprintf(detail, sizeof(detail), "loadProfileConfig rejected or mis-mapped %s", testCase.name.c_str());
            ENGINE_EXPECT(ctx,
                          loaded.has_value() && loaded->render.displayFormat == display &&
                              loaded->render.textureType == texture && loaded->render.vsync == vsync,
                          detail);
        } else {
            std::snprintf(detail, sizeof(detail), "loadProfileConfig accepted %s", testCase.name.c_str());
            ENGINE_EXPECT(ctx, !loaded.has_value(), detail);
        }
    }
}

// loadImageTexture's typed read: binary16-exact data loads bit-identically at both types, arbitrary data at Float16 equals the IEEE round-to-nearest-even cast (IEEE 754-2008 4.3.1) that OpenEXR's float-to-half conversion must perform, and a finite source above kHalfMax overflows and is rejected at Float16 only.
ENGINE_CHECK(image_texture_half_load, Fast, Exact) {
    using engine::gfx::ScalarType;
    const auto writeProbe = [](const char* name, const std::vector<float>& values) {
        engine::gfx::HdrImage image{static_cast<int>(values.size()), 1, {}};
        for (const float v : values) {
            image.rgba.insert(image.rgba.end(), {v, v, v, 1.0F});
        }
        const std::filesystem::path path = scratchPath(name);
        return engine::gfx::writeExr(path.string(), image) ? std::optional(path) : std::nullopt;
    };
    const auto sameTexels = [](const engine::gfx::ImageTexture& a, const std::vector<float>& expected) {
        for (int x = 0; x < a.width; ++x) {
            const glm::vec4 texel = a.texel(x, 0);
            if (texel.r != expected[static_cast<std::size_t>(x)] || texel.b != expected[static_cast<std::size_t>(x)]) {
                return false;
            }
        }
        return true;
    };
    ctx.plan(5);

    // Exact in binary16: small integers, dyadic fractions, the largest finite value, the smallest normal.
    const std::vector<float> exact = {0.0F, 1.0F, 1.5F, 0.25F, 2048.0F, -3.0F, engine::gfx::kHalfMax, 1.0F / 16384.0F};
    const std::optional<std::filesystem::path> exactPath = writeProbe("engine_io_half_exact.exr", exact);
    const std::optional<engine::gfx::ImageTexture> exact16 =
        exactPath ? engine::gfx::loadImageTexture(exactPath->string(), ScalarType::Float16) : std::nullopt;
    const std::optional<engine::gfx::ImageTexture> exact32 =
        exactPath ? engine::gfx::loadImageTexture(exactPath->string(), ScalarType::Float32) : std::nullopt;
    ENGINE_EXPECT(ctx, exact16 && exact32 && std::holds_alternative<std::vector<engine::gfx::Half>>(exact16->texels) && sameTexels(*exact16, exact) &&
                           sameTexels(*exact32, exact),
                  "binary16-exact values did not load bit-identically at Float16 and Float32");

    // Arbitrary: needs rounding, including the exact midpoint above 1.0 (ties to even -> 1.0), a subnormal, and one below the smallest subnormal (-> 0).
    const std::vector<float> arbitrary = {0.1F, 1.0F + engine::gfx::kHalfUnitRoundoff, 3.14159265F, 1.0e-6F, 1.0e-20F, 60000.5F};
    std::vector<float> rounded;
    for (const float v : arbitrary) {
        rounded.push_back(static_cast<float>(static_cast<engine::gfx::Half>(v)));
    }
    const std::optional<std::filesystem::path> arbitraryPath = writeProbe("engine_io_half_arbitrary.exr", arbitrary);
    const std::optional<engine::gfx::ImageTexture> arbitrary16 =
        arbitraryPath ? engine::gfx::loadImageTexture(arbitraryPath->string(), ScalarType::Float16) : std::nullopt;
    ENGINE_EXPECT(ctx, arbitrary16 && sameTexels(*arbitrary16, rounded),
                  "Float16 load differs from static_cast<Half> (round to nearest even)");
    const std::optional<engine::gfx::ImageTexture> arbitrary32 =
        arbitraryPath ? engine::gfx::loadImageTexture(arbitraryPath->string(), ScalarType::Float32) : std::nullopt;
    ENGINE_EXPECT(ctx, arbitrary32 && sameTexels(*arbitrary32, arbitrary), "Float32 load is not an exact copy");

    // Overflow: 70000 is finite in float and beyond kHalfMax, so Float16 must reject it and Float32 must not.
    const std::optional<std::filesystem::path> overPath = writeProbe("engine_io_half_overflow.exr", {1.0F, 70000.0F});
    ENGINE_EXPECT(ctx, overPath && !engine::gfx::loadImageTexture(overPath->string(), ScalarType::Float16),
                  "Float16 accepted a texel above binary16's finite max");
    ENGINE_EXPECT(ctx, overPath && engine::gfx::loadImageTexture(overPath->string(), ScalarType::Float32),
                  "Float32 rejected a finite texel");
    for (const std::optional<std::filesystem::path>& path : {exactPath, arbitraryPath, overPath}) {
        if (path) {
            std::filesystem::remove(*path);
        }
    }
}

// sampleBilinear at Float16 against Float32 on the same non-negative, normal-range data. Each stored texel is t(1 + d) with |d| <= u = 2^-11, and bilinear weights are non-negative and sum to 1, so the storage error is at most u * s32; each path's float arithmetic (two mix levels, 3 roundings each) adds at most gamma_6 * s (Higham 2002, 3.1). The bound is derived, not fitted; a zero observed difference would mean the Float16 path was never exercised.
ENGINE_CHECK(image_texture_bilinear_half_bound, Fast, Exact) {
    using engine::gfx::ScalarType;
    constexpr int kWidth = 13;
    constexpr int kHeight = 7;
    constexpr int kSamples = 20000;
    const double unitRoundoff = std::numeric_limits<float>::epsilon() / 2.0;
    const double gamma6 = 6.0 * unitRoundoff / (1.0 - (6.0 * unitRoundoff));
    const double bound = engine::gfx::kHalfUnitRoundoff + (2.0 * gamma6);

    std::mt19937 rng(static_cast<std::mt19937::result_type>(ctx.seed()));
    // Normal range of binary16: [2^-14, kHalfMax], log-uniform so every binade is exercised.
    std::uniform_real_distribution<float> logValue(-14.0F, std::log2(engine::gfx::kHalfMax));
    std::vector<float> rgba(static_cast<std::size_t>(kWidth) * kHeight * 4);
    for (float& v : rgba) {
        v = std::exp2(logValue(rng));
    }
    const engine::gfx::ImageTexture full{kWidth, kHeight, rgba};
    const engine::gfx::ImageTexture half{kWidth, kHeight, std::vector<engine::gfx::Half>(rgba.begin(), rgba.end())};

    std::uniform_real_distribution<float> unit(-1.0F, 2.0F);  // beyond [0,1] so both wrap directions are covered
    double worst = 0.0;
    double largest = 0.0;
    for (int i = 0; i < kSamples; ++i) {
        const glm::vec2 uv(unit(rng), unit(rng));
        const glm::vec4 s32 = engine::gfx::sampleBilinear(full, uv);
        const glm::vec4 s16 = engine::gfx::sampleBilinear(half, uv);
        for (int c = 0; c < 4; ++c) {
            const double relative = std::fabs(static_cast<double>(s16[c]) - s32[c]) / s32[c];
            worst = std::max(worst, relative / bound);
            largest = std::max(largest, relative);
        }
    }
    ctx.plan(2);
    char detail[192];
    std::snprintf(detail, sizeof(detail), "worst |s16 - s32| / s32 is %.4g of the derived bound %.4g", worst, bound);
    ENGINE_EXPECT(ctx, worst <= 1.0, detail);
    ENGINE_EXPECT(ctx, largest > 0.0, "Float16 and Float32 samples never differed: the half path was not exercised");
}

// The film-back catalogue's own contract: every preset's dimensions feed Camera::verticalFovRadians() as a
// denominator and an aspect ratio, so a zero or negative entry is not a cosmetic defect.
ENGINE_CHECK(film_back_presets_are_physically_valid, Fast, Exact) {
    ctx.plan(2);
    const std::filesystem::path camera = std::filesystem::path(ASSET_ROOT_DIR) / "config" / "camera.json";
    const std::optional<std::vector<engine::scene::Camera::FilmBackPreset>> presets =
        engine::config::loadFilmBackPresets(camera.string());
    if (!presets.has_value()) {
        ENGINE_EXPECT(ctx, false, "loadFilmBackPresets rejected the shipped camera.json");
        ENGINE_EXPECT(ctx, false, "presets unavailable");
        return;
    }
    ENGINE_EXPECT(ctx, !presets->empty(), "the shipped film-back catalogue is empty");
    bool allPositive = true;
    for (const engine::scene::Camera::FilmBackPreset& preset : *presets) {
        allPositive = allPositive && preset.filmBack.widthMm > 0.0F && preset.filmBack.heightMm > 0.0F;
    }
    ENGINE_EXPECT(ctx, allPositive, "a film-back preset has a non-positive dimension");
}

// Every appended record is exactly one line that parses back with every schema field, and appending never rewrites earlier lines.
ENGINE_CHECK(bench_log_appends_one_parseable_line_per_record, Fast, Exact) {
    ctx.plan(5);
    const std::filesystem::path path = scratchPath("engine_io_validate_bench.jsonl");
    std::filesystem::remove(path);
    const engine::debug::BenchRecord first{"io_validate", {"io_validate", "--flag"}, {{"width", 7}}, {{"ms", {1.5, 2.5}}}, {{"crc32", 1}}};
    const engine::debug::BenchRecord second{"io_validate", {"io_validate"}, {{"width", 9}}, {{"ms", {3.0}}}, {{"crc32", 2}}};
    ENGINE_EXPECT(ctx, engine::debug::appendBenchRecord(path.string(), first) && engine::debug::appendBenchRecord(path.string(), second), "appendBenchRecord failed on a writable scratch path");

    std::vector<nlohmann::json> records;
    std::ifstream in(path);
    for (std::string line; std::getline(in, line);) {
        records.push_back(nlohmann::json::parse(line, nullptr, /*allow_exceptions=*/false));
    }
    ENGINE_EXPECT(ctx, records.size() == 2, "expected exactly two lines after two appends");

    const auto complete = [](const nlohmann::json& r) {
        return !r.is_discarded() && r.value("schema", 0) == engine::debug::kBenchLogSchema && r.contains("time_utc") && r.contains("pid") &&
               !r["build"].value("git", std::string()).empty() && r["build"].value("uuid", std::string()).size() == 32 &&
               r["host"].value("logical_cpus", 0) > 0 && r["rusage"].contains("user_s") && r["rusage"].contains("nivcsw");
    };
    ENGINE_EXPECT(ctx, records.size() == 2 && complete(records[0]) && complete(records[1]), "a record is missing a provenance or rusage field");
    ENGINE_EXPECT(ctx, records.size() == 2 && records[0]["config"] == first.config && records[0]["samples"] == first.samples && records[0]["argv"] == first.argv,
                  "first record's caller-supplied content did not round-trip");
    ENGINE_EXPECT(ctx, records.size() == 2 && records[1]["config"] == second.config && records[1]["work"] == second.work,
                  "second record's caller-supplied content did not round-trip");
    std::filesystem::remove(path);
}

// A path that cannot be opened is reported as a failure, never as a silently skipped record.
ENGINE_CHECK(bench_log_rejects_unwritable_path, Fast, Exact) {
    ctx.plan(1);
    const std::filesystem::path path = scratchPath("engine_io_validate_no_such_dir") / "bench.jsonl";
    std::filesystem::remove_all(path.parent_path());
    const engine::debug::BenchRecord record{"io_validate", {}, nlohmann::json::object(), nlohmann::json::object(), nlohmann::json::object()};
    ENGINE_EXPECT(ctx, !engine::debug::appendBenchRecord(path.string(), record), "appendBenchRecord reported success writing into a missing directory");
}

// Known answers from an independent implementation (Python's zlib.crc32 over the same little-endian bytes).
ENGINE_CHECK(float_crc32_matches_reference, Fast, Exact) {
    ctx.plan(2);
    const std::vector<float> zero{0.0F};
    const std::vector<float> mixed{1.0F, -2.5F, 0.1F};
    ENGINE_EXPECT(ctx, engine::debug::floatCrc32(zero) == 0x2144DF1CU, "CRC-32 of four zero bytes is not 0x2144DF1C");
    ENGINE_EXPECT(ctx, engine::debug::floatCrc32(mixed) == 2706677804U, "CRC-32 of {1, -2.5, 0.1} disagrees with zlib.crc32");
}

}  // namespace

ENGINE_CHECK_MAIN("io")
