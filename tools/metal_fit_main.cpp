// Prints the reflectivity/edgeTint a material JSON should carry for a measured conductor, at max_digits10 so pasting round-trips exactly.

#include <cstdio>
#include <iostream>
#include <limits>
#include <string_view>

#include "metal_fit.h"

namespace {

using tools::metal_fit::Fit;
using tools::metal_fit::Interpolation;

void printFloatTriple(const char* key, const glm::dvec3& v) {
    constexpr int kDigits = std::numeric_limits<float>::max_digits10;
    std::printf("    \"%s\": [%.*g, %.*g, %.*g],\n", key, kDigits, static_cast<float>(v.x), kDigits,
                static_cast<float>(v.y), kDigits, static_cast<float>(v.z));
}

void printTriple(const char* label, const glm::dvec3& v) {
    std::printf("  %-44s %.3e %.3e %.3e\n", label, v.x, v.y, v.z);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 3 || std::string_view(argv[1]) != "--nk") {
        std::cerr << "usage: metal_fit --nk TABLE.csv   (rows lambda_um,n,k spanning 360-830 nm)\n";
        return 2;
    }
    const auto table = tools::metal_fit::loadNkTable(argv[2]);
    if (!table) {
        return 1;
    }
    const auto fit = tools::metal_fit::fitGulbrandsen(*table, Interpolation::Wavelength);
    const auto energyFit = tools::metal_fit::fitGulbrandsen(*table, Interpolation::PhotonEnergy);
    if (!fit || !energyFit) {
        return 1;
    }
    std::printf("metal_fit: %s (%zu rows) -> CIE 1931 2-degree observer, D65, linear Rec.709\n", argv[2],
                table->size());
    printFloatTriple("diffuseColour", fit->reflectivity);
    printFloatTriple("edgeTint", fit->edgeTint);
    printTriple("hemispherical average (fitted exactly)", fit->averageTarget);
    printTriple("max |R_fit(mu) - R_cie(mu)|", fit->angularResidual);
    printTriple("reflectivity delta, linear in photon energy", energyFit->reflectivity - fit->reflectivity);
    printTriple("edgeTint delta, linear in photon energy", energyFit->edgeTint - fit->edgeTint);
    return 0;
}
