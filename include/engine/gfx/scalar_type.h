#pragma once

#include <cstddef>
#include <optional>

namespace engine::gfx {

// IEEE 754 binary16, the compiler's native half: AArch64 widens it with one FCVT (FCVTL for 4 lanes), unlike Imath::half's 256 KB lookup table, and its bit layout is OpenEXR's HALF.
using Half = _Float16;
static_assert(sizeof(Half) == 2, "binary16 must be 2 bytes to alias OpenEXR HALF data");

// binary16 format parameters (IEEE 754-2008 Table 3.2): precision p = 11, emax = 15; derived here because libc++ does not specialise numeric_limits<_Float16>.
inline constexpr int kHalfPrecision = 11;
inline constexpr int kHalfMaxExponent = 15;
inline constexpr float kHalfMax = (2.0F - (1.0F / static_cast<float>(1 << (kHalfPrecision - 1)))) * static_cast<float>(1 << kHalfMaxExponent);
inline constexpr float kHalfUnitRoundoff = 1.0F / static_cast<float>(1 << kHalfPrecision);  // max relative error of round-to-nearest
static_assert(kHalfMax == 65504.0F);

// Storage precision of image data: Float16 (11-bit significand, unit roundoff 2^-11, finite max 65504) or Float32 (exact copy of the float source).
enum class ScalarType { Float16, Float32 };

[[nodiscard]] constexpr std::size_t scalarBytes(ScalarType type) {
    switch (type) {
        case ScalarType::Float16:
            return sizeof(Half);
        case ScalarType::Float32:
            return sizeof(float);
    }
    return 0;
}

[[nodiscard]] constexpr const char* scalarTypeName(ScalarType type) {
    switch (type) {
        case ScalarType::Float16:
            return "float16";
        case ScalarType::Float32:
            return "float32";
    }
    return "";
}

// profile.json's bit-depth vocabulary; nullopt for any depth without a float storage type.
[[nodiscard]] constexpr std::optional<ScalarType> scalarTypeFromBitDepth(int bitDepth) {
    switch (bitDepth) {
        case 16:
            return ScalarType::Float16;
        case 32:
            return ScalarType::Float32;
        default:
            return std::nullopt;
    }
}

}  // namespace engine::gfx
