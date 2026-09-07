#include "engine/debug/power_spectrum.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <numeric>

namespace engine::debug {

namespace {

// DFT of one strided line of a complex field, in place. The running `angle` replaces a `(k * t) % n` per tap: the
// twiddle index advances by k and wraps, which is the same value without the division that would otherwise dominate.
void dftLine(double* re, double* im, int n, std::ptrdiff_t stride, const std::vector<double>& cosTable,
             const std::vector<double>& sinTable, std::vector<double>& scratchRe, std::vector<double>& scratchIm) {
    for (int k = 0; k < n; ++k) {
        double sumRe = 0.0;
        double sumIm = 0.0;
        int angle = 0;
        for (int t = 0; t < n; ++t) {
            const double c = cosTable[static_cast<std::size_t>(angle)];
            const double s = sinTable[static_cast<std::size_t>(angle)];
            const double vr = re[static_cast<std::ptrdiff_t>(t) * stride];
            const double vi = im[static_cast<std::ptrdiff_t>(t) * stride];
            sumRe += (vr * c) - (vi * s);
            sumIm += (vr * s) + (vi * c);
            angle += k;
            angle -= angle >= n ? n : 0;
        }
        scratchRe[static_cast<std::size_t>(k)] = sumRe;
        scratchIm[static_cast<std::size_t>(k)] = sumIm;
    }
    for (int k = 0; k < n; ++k) {
        re[static_cast<std::ptrdiff_t>(k) * stride] = scratchRe[static_cast<std::size_t>(k)];
        im[static_cast<std::ptrdiff_t>(k) * stride] = scratchIm[static_cast<std::size_t>(k)];
    }
}

void makeTwiddle(int n, std::vector<double>& cosTable, std::vector<double>& sinTable) {
    cosTable.resize(static_cast<std::size_t>(n));
    sinTable.resize(static_cast<std::size_t>(n));
    for (int a = 0; a < n; ++a) {
        const double theta = -2.0 * std::numbers::pi * a / n;  // forward transform, exp(-i theta)
        cosTable[static_cast<std::size_t>(a)] = std::cos(theta);
        sinTable[static_cast<std::size_t>(a)] = std::sin(theta);
    }
}

// Which octave a lattice point falls in, band 0 being the top one (Nyquist/2 to Nyquist). Axis frequencies are
// normalised independently, so a non-square field bins by true radial frequency in cycles/pixel. The clamp folds the
// corners past Nyquist into band 0 and anything below the eighth octave into the last band, so every point is counted
// exactly once and octaveBandPower and whiteNoiseBandShare cannot disagree about where a point belongs.
int bandOf(int kx, int ky, int width, int height) {
    const double fx = static_cast<double>(kx <= width / 2 ? kx : kx - width) / width;
    const double fy = static_cast<double>(ky <= height / 2 ? ky : ky - height) / height;
    const double r = std::sqrt((fx * fx) + (fy * fy));
    if (r == 0.0) {
        return -1;  // DC, which carries the mean rather than any arrangement of the field
    }
    return std::clamp(static_cast<int>(std::floor(std::log2(0.5 / r))), 0, kSpectrumBands - 1);
}

}  // namespace

std::array<double, kSpectrumBands> octaveBandPower(const std::vector<double>& field, int width, int height) {
    std::vector<double> re = field;
    std::vector<double> im(re.size(), 0.0);
    const double mean = std::accumulate(re.begin(), re.end(), 0.0) / static_cast<double>(re.size());
    for (double& v : re) {
        v -= mean;
    }

    std::vector<double> cosTable;
    std::vector<double> sinTable;
    std::vector<double> scratchRe(static_cast<std::size_t>(std::max(width, height)));
    std::vector<double> scratchIm(scratchRe.size());
    makeTwiddle(width, cosTable, sinTable);
    for (int y = 0; y < height; ++y) {
        const auto row = static_cast<std::size_t>(y) * static_cast<std::size_t>(width);
        dftLine(&re[row], &im[row], width, 1, cosTable, sinTable, scratchRe, scratchIm);
    }
    makeTwiddle(height, cosTable, sinTable);
    for (int x = 0; x < width; ++x) {
        dftLine(&re[static_cast<std::size_t>(x)], &im[static_cast<std::size_t>(x)], height, width, cosTable, sinTable,
                 scratchRe, scratchIm);
    }

    std::array<double, kSpectrumBands> bands{};
    for (int ky = 0; ky < height; ++ky) {
        for (int kx = 0; kx < width; ++kx) {
            const int band = bandOf(kx, ky, width, height);
            if (band < 0) {
                continue;
            }
            const auto i = (static_cast<std::size_t>(ky) * static_cast<std::size_t>(width)) +
                            static_cast<std::size_t>(kx);
            bands[static_cast<std::size_t>(band)] += (re[i] * re[i]) + (im[i] * im[i]);
        }
    }
    return bands;
}

std::array<double, kSpectrumBands> whiteNoiseBandShare(int width, int height) {
    std::array<double, kSpectrumBands> shares{};
    double total = 0.0;
    for (int ky = 0; ky < height; ++ky) {
        for (int kx = 0; kx < width; ++kx) {
            const int band = bandOf(kx, ky, width, height);
            if (band < 0) {
                continue;
            }
            shares[static_cast<std::size_t>(band)] += 1.0;
            total += 1.0;
        }
    }
    for (double& share : shares) {
        share /= total;
    }
    return shares;
}

}  // namespace engine::debug
