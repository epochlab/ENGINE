// Offline generator for the blue-noise dither mask baked into src/scene/blue_noise_mask.inc, which sampler.cpp uses as
// the per-pixel Cranley-Patterson shift of Georgiev & Fajardo's blue-noise dithered sampling (SIGGRAPH 2016 Talks).
// The algorithm is Ulichney's void-and-cluster method, "The void-and-cluster method for dither array generation", Proc.
// SPIE 1913 (1993), transcribed from the author's own paper -- filter, generator and all three ranking phases below name
// the section they come from. It is in-repo and deterministic rather than a lifted third-party tile precisely so the
// provenance is an algorithm that can be re-run and checked, not a binary blob whose construction cannot be audited.
// Same standalone-CLI convention as the other tools: no test framework, non-zero exit on failure. Not in the ctest loop
// -- like gltf_tangent this produces a committed artifact, it does not check one; src/scene/blue_noise_mask.inc is the
// output and sampler_validate is what holds it to account.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "engine/gfx/hdr_image.h"

namespace {

// 128^2, the size Georgiev & Fajardo used for the images in the paper ("we plot matrices of size 64^2, but we used size
// 128^2 to render our images"). Must match kMaskSize in sampler.cpp.
constexpr int kSize = 128;
constexpr int kPixels = kSize * kSize;

// Ulichney Sec. 2: the filter is a Gaussian whose "sigma in terms of pixel spacing produced the best results" at 1.5.
// Not truncated to a radius -- a cutoff would be a free parameter, and the wrap-around form below is what makes the
// resulting array tileable, which is the property sampler.cpp depends on when it indexes the mask modulo kSize.
constexpr double kSigma = 1.5;

// Ulichney Sec. 3: the initial binary pattern starts from white noise whose 1s are a small minority (his Fig. 3 uses
// 26 of 256, ~10%); the void-and-cluster loop then rearranges them into a well-formed blue-noise pattern, so this
// governs how much rearranging is needed, not the quality of the result.
constexpr int kInitialOnes = kPixels / 10;

// Energy field of a set of marked cells: the pattern convolved with the Gaussian above. "Tightest cluster" is the marked
// cell sitting in the most energy, "largest void" the unmarked cell sitting in the least -- Ulichney's two primitives,
// and every phase below is a loop over them.
// Both phases I/II (marked = the 1s) and phase III (marked = the 0s) are the same search over a different marked set, so
// there is one class here rather than a second inverted copy of it.
class EnergyField {
public:
    EnergyField() {
        // Indexed by wrapped offset, so insert/erase is a shifted read of one table rather than a per-cell exp().
        // Ulichney Sec. 2's wrap-around convolution: the offset that counts is the minimum-image one on the torus.
        for (int dy = 0; dy < kSize; ++dy) {
            const int wy = std::min(dy, kSize - dy);
            for (int dx = 0; dx < kSize; ++dx) {
                const int wx = std::min(dx, kSize - dx);
                const double squaredDistance = static_cast<double>((wx * wx) + (wy * wy));
                kernel_[(static_cast<std::size_t>(dy) * kSize) + static_cast<std::size_t>(dx)] =
                    std::exp(-squaredDistance / (2.0 * kSigma * kSigma));
            }
        }
    }

    // Rebuilt from the pattern rather than rewound by undoing inserts: phases I and II both restart from the initial
    // binary pattern, and recomputing costs one pass where rewinding would carry every phase's rounding into the next.
    void reset(const std::array<std::uint8_t, kPixels>& marked) {
        marked_ = marked;
        markedCount_ = 0;
        energy_.fill(0.0);
        for (int i = 0; i < kPixels; ++i) {
            if (marked_[static_cast<std::size_t>(i)] != 0) {
                splat(i, 1.0);
                ++markedCount_;
            }
        }
    }

    void insert(int i) {
        marked_[static_cast<std::size_t>(i)] = 1;
        splat(i, 1.0);
        ++markedCount_;
    }

    void erase(int i) {
        marked_[static_cast<std::size_t>(i)] = 0;
        splat(i, -1.0);
        --markedCount_;
    }

    [[nodiscard]] int tightestCluster() const { return extremum(1, /*wantMax=*/true); }
    [[nodiscard]] int largestVoid() const { return extremum(0, /*wantMax=*/false); }
    [[nodiscard]] int markedCount() const { return markedCount_; }
    [[nodiscard]] const std::array<std::uint8_t, kPixels>& marked() const { return marked_; }

private:
    // Adds `sign` times the kernel centred on i. The x offset is advanced and wrapped rather than recomputed with a
    // modulo per cell: same value, and the modulo is the dominant cost of the whole generator otherwise.
    void splat(int i, double sign) {
        const int iy = i / kSize;
        const int ix = i % kSize;
        for (int jy = 0; jy < kSize; ++jy) {
            const int dy = (jy - iy + kSize) % kSize;
            const double* kernelRow = &kernel_[static_cast<std::size_t>(dy) * kSize];
            double* energyRow = &energy_[static_cast<std::size_t>(jy) * kSize];
            int dx = (kSize - ix) % kSize;
            for (int jx = 0; jx < kSize; ++jx) {
                energyRow[jx] += sign * kernelRow[dx];
                dx = dx + 1 == kSize ? 0 : dx + 1;
            }
        }
    }

    // Ties resolve to the lowest index, which is what makes the whole generator reproducible: the search order is the
    // only thing distinguishing two cells at exactly equal energy.
    [[nodiscard]] int extremum(std::uint8_t state, bool wantMax) const {
        int best = -1;
        double bestEnergy = 0.0;
        for (int i = 0; i < kPixels; ++i) {
            if (marked_[static_cast<std::size_t>(i)] != state) {
                continue;
            }
            const double e = energy_[static_cast<std::size_t>(i)];
            if (best < 0 || (wantMax ? e > bestEnergy : e < bestEnergy)) {
                best = i;
                bestEnergy = e;
            }
        }
        return best;
    }

    std::array<double, kPixels> kernel_{};
    std::array<double, kPixels> energy_{};
    std::array<std::uint8_t, kPixels> marked_{};
    int markedCount_ = 0;
};

// Ulichney Sec. 3 / Fig. 2. White noise in, then: remove the tightest cluster, find the largest void, and stop when
// removing that 1 is what created the largest void -- the fixed point where no 1 can be moved anywhere better. The
// result is the "initial binary pattern" all three ranking phases start from.
// The shuffle is written out rather than taken from <algorithm>/<random>'s distributions because only mt19937's raw
// output is specified exactly by the standard; std::shuffle and uniform_int_distribution are free to vary between
// implementations, and a mask that regenerates differently on another compiler is not a re-derivable committed artifact.
std::array<std::uint8_t, kPixels> initialBinaryPattern(std::uint32_t seed, EnergyField& field) {
    std::array<int, kPixels> order{};
    std::iota(order.begin(), order.end(), 0);
    std::mt19937 rng(seed);
    for (int i = kPixels - 1; i > 0; --i) {
        std::swap(order[static_cast<std::size_t>(i)], order[rng() % static_cast<std::uint32_t>(i + 1)]);
    }

    std::array<std::uint8_t, kPixels> pattern{};
    for (int i = 0; i < kInitialOnes; ++i) {
        pattern[static_cast<std::size_t>(order[static_cast<std::size_t>(i)])] = 1;
    }

    field.reset(pattern);
    for (;;) {
        const int cluster = field.tightestCluster();
        field.erase(cluster);
        const int emptiest = field.largestVoid();
        if (emptiest == cluster) {
            field.insert(cluster);  // removing this 1 is what opened the largest void -- converged
            return field.marked();
        }
        field.insert(emptiest);
    }
}

// Ulichney Sec. 4's three phases, which between them assign every cell a distinct rank in [0, kPixels).
// I: from the initial pattern, remove tightest clusters, ranking each by the count of 1s left after it goes.
// II: from the initial pattern again, fill largest voids, ranking each by the count of 1s before it arrives.
// III: past half density the 0s are the minority, so the roles swap -- fill the tightest cluster of 0s instead, which is
// the same search over the complement and is why EnergyField is written in terms of a marked set.
std::array<std::uint16_t, kPixels> rankPattern(const std::array<std::uint8_t, kPixels>& initial, EnergyField& field) {
    std::array<std::uint16_t, kPixels> ranks{};

    field.reset(initial);
    while (field.markedCount() > 0) {
        const int i = field.tightestCluster();
        field.erase(i);
        ranks[static_cast<std::size_t>(i)] = static_cast<std::uint16_t>(field.markedCount());
    }

    field.reset(initial);
    while (field.markedCount() <= kPixels / 2) {
        const int i = field.largestVoid();
        ranks[static_cast<std::size_t>(i)] = static_cast<std::uint16_t>(field.markedCount());
        field.insert(i);
    }

    std::array<std::uint8_t, kPixels> complement{};
    for (int i = 0; i < kPixels; ++i) {
        complement[static_cast<std::size_t>(i)] = field.marked()[static_cast<std::size_t>(i)] ^ 1U;
    }
    int ones = kPixels - std::accumulate(complement.begin(), complement.end(), 0);
    field.reset(complement);
    while (field.markedCount() > 0) {
        const int i = field.tightestCluster();
        field.erase(i);
        ranks[static_cast<std::size_t>(i)] = static_cast<std::uint16_t>(ones);
        ++ones;
    }
    return ranks;
}

// The one property the rest of the system assumes: the mask is a bijection onto [0, kPixels), so its values are exactly
// the uniform grid the toroidal shift needs. Checked here so a malformed table can never reach the committed .inc.
bool isPermutation(const std::array<std::uint16_t, kPixels>& ranks) {
    std::array<std::uint8_t, kPixels> seen{};
    for (const std::uint16_t rank : ranks) {
        if (rank >= kPixels || seen[rank] != 0) {
            return false;
        }
        seen[rank] = 1;
    }
    return true;
}

bool writeInc(const std::string& path, const std::array<std::uint16_t, kPixels>& ranks, std::uint32_t seed) {
    std::ofstream out(path);
    if (!out) {
        std::cerr << "bluenoise_mask: cannot write " << path << "\n";
        return false;
    }
    out << "// Generated by tools/bluenoise_mask.cpp -- do not edit. Regenerate with:\n"
        << "//   ./build/bluenoise_mask --out src/scene/blue_noise_mask.inc\n"
        << "// Void-and-cluster dither array (Ulichney 1993), " << kSize << "x" << kSize << ", sigma " << kSigma
        << ", initial pattern seed " << seed << ".\n"
        << "// Row-major ranks, a permutation of [0, " << kPixels << "). sampler.cpp turns rank r into the toroidal\n"
        << "// shift (r + 0.5) / " << kPixels << ".\n";
    for (int i = 0; i < kPixels; ++i) {
        out << (i % 16 == 0 ? "\n" : " ") << ranks[static_cast<std::size_t>(i)] << ",";
    }
    out << "\n";
    return out.good();
}

// Greyscale dump of the normalised mask, for looking at. The radial spectrum in sampler_validate is the actual gate --
// this catches the structural failures an angular average hides, like a directional streak or a residual tile seam.
bool writePreview(const std::string& path, const std::array<std::uint16_t, kPixels>& ranks) {
    engine::gfx::HdrImage image{kSize, kSize, std::vector<float>(static_cast<std::size_t>(kPixels) * 4, 1.0F)};
    for (int i = 0; i < kPixels; ++i) {
        const auto value = static_cast<float>((ranks[static_cast<std::size_t>(i)] + 0.5) / kPixels);
        for (int c = 0; c < 3; ++c) {
            image.rgba[(static_cast<std::size_t>(i) * 4) + static_cast<std::size_t>(c)] = value;
        }
    }
    return engine::gfx::writeExr(path, image);
}

}  // namespace

int main(int argc, char** argv) {
    std::string outPath = "src/scene/blue_noise_mask.inc";
    std::string previewPath;
    std::uint32_t seed = 1;
    for (int i = 1; i < argc; ++i) {
        const bool hasValue = i + 1 < argc;
        if (std::strcmp(argv[i], "--out") == 0 && hasValue) {
            outPath = argv[++i];
        } else if (std::strcmp(argv[i], "--preview") == 0 && hasValue) {
            previewPath = argv[++i];
        } else if (std::strcmp(argv[i], "--seed") == 0 && hasValue) {
            seed = static_cast<std::uint32_t>(std::strtoul(argv[++i], nullptr, 10));
        } else {
            std::cerr << "bluenoise_mask: unknown or incomplete argument '" << argv[i]
                      << "'\nusage: bluenoise_mask [--out path.inc] [--preview path.exr] [--seed N]\n";
            return EXIT_FAILURE;
        }
    }

    EnergyField field;
    const std::array<std::uint8_t, kPixels> initial = initialBinaryPattern(seed, field);
    const std::array<std::uint16_t, kPixels> ranks = rankPattern(initial, field);
    if (!isPermutation(ranks)) {
        std::cerr << "bluenoise_mask: ranks are not a permutation of [0, " << kPixels << ") -- generator is wrong\n";
        return EXIT_FAILURE;
    }
    if (!writeInc(outPath, ranks, seed)) {
        return EXIT_FAILURE;
    }
    std::cout << "bluenoise_mask: wrote " << outPath << " (" << kSize << "x" << kSize << ", sigma " << kSigma
              << ", seed " << seed << ")\n";
    if (!previewPath.empty() && !writePreview(previewPath, ranks)) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
