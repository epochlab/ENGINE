#include "metal_fit.h"

#include <cassert>
#include <cmath>
#include <complex>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>

#include "conductor_reference.h"
#include "pathtracer/scene/cie.h"

namespace tools::metal_fit {

namespace {

namespace cie = pathtracer::scene::cie;
using tools::reference::cosineAverageFresnel;
using tools::reference::referenceConductorFresnel;
using tools::reference::referenceConductorFresnelAt;

// Gulbrandsen's r domain as bsdf.cpp clamps it (kMin/kMaxReflectivity); a target outside it is silently clamped at render time.
constexpr double kMinReflectivity = 1e-4;
constexpr double kMaxReflectivity = 0.9999;

// Uniform mu grid the angular residual is reported on.
constexpr int kResidualAngles = 1001;

// Complex IOR on the CIE grid, interpolating n and k (never R, which is nonlinear in them) between bracketing rows.
std::vector<std::complex<double>> sampleIor(const std::vector<NkSample>& table, Interpolation interpolation) {
    std::vector<std::complex<double>> eta(cie::kSampleCount);
    const auto axis = [interpolation](double lambdaNm) {
        return interpolation == Interpolation::Wavelength ? lambdaNm : 1.0 / lambdaNm;
    };
    std::size_t row = 0;
    for (int i = 0; i < cie::kSampleCount; ++i) {
        const double lambda = cie::wavelengthNm(i);
        while (table[row + 1].lambdaNm < lambda) {
            ++row;
        }
        const NkSample& a = table[row];
        const NkSample& b = table[row + 1];
        const double t = (axis(lambda) - axis(a.lambdaNm)) / (axis(b.lambdaNm) - axis(a.lambdaNm));
        eta[i] = {std::lerp(a.n, b.n, t), std::lerp(a.k, b.k, t)};
    }
    return eta;
}

template <typename Reflectance>
glm::dvec3 projectSpectrum(const std::vector<std::complex<double>>& eta, Reflectance reflectance) {
    cie::Spectrum spectrum;
    for (int i = 0; i < cie::kSampleCount; ++i) {
        spectrum[i] = reflectance(eta[i]);
    }
    return cie::reflectanceToRec709(spectrum);
}

double modelAverage(double r, double g) {
    return cosineAverageFresnel([&](double mu) { return referenceConductorFresnel(r, g, mu); });
}

// Bisection on g in [0, 1]; digits halvings resolve g to one ulp at g = 1.
double solveEdgeTint(double r, double target) {
    double lo = 0.0;
    double hi = 1.0;
    for (int i = 0; i < std::numeric_limits<double>::digits; ++i) {
        const double mid = 0.5 * (lo + hi);
        (modelAverage(r, mid) < target ? lo : hi) = mid;
    }
    return 0.5 * (lo + hi);
}

}  // namespace

std::optional<std::vector<NkSample>> loadNkTable(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        std::cerr << "loadNkTable: cannot read " << path << '\n';
        return std::nullopt;
    }
    std::vector<NkSample> table;
    std::string line;
    for (int lineNumber = 1; std::getline(in, line); ++lineNumber) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();  // CRLF tables from Windows-authored sources
        }
        if (line.empty() || line.front() == '#') {
            continue;
        }
        double lambdaUm = 0.0;
        NkSample sample{};
        char tail = 0;
        if (std::sscanf(line.c_str(), "%lf,%lf,%lf %c", &lambdaUm, &sample.n, &sample.k, &tail) != 3) {
            std::cerr << "loadNkTable: " << path << ':' << lineNumber << ": expected lambda_um,n,k\n";
            return std::nullopt;
        }
        sample.lambdaNm = lambdaUm * 1000.0;
        if (!(sample.n > 0.0 && sample.k >= 0.0)) {
            std::cerr << "loadNkTable: " << path << ':' << lineNumber << ": needs n > 0 and k >= 0\n";
            return std::nullopt;
        }
        if (!table.empty() && !(sample.lambdaNm > table.back().lambdaNm)) {
            std::cerr << "loadNkTable: " << path << ':' << lineNumber << ": wavelengths must strictly increase\n";
            return std::nullopt;
        }
        table.push_back(sample);
    }
    if (table.empty() || table.front().lambdaNm > cie::kLambdaMinNm || table.back().lambdaNm < cie::kLambdaMaxNm) {
        std::cerr << "loadNkTable: " << path << ": must span " << cie::kLambdaMinNm << "-" << cie::kLambdaMaxNm
                  << " nm\n";
        return std::nullopt;
    }
    return table;
}

std::optional<Fit> fitGulbrandsen(const std::vector<NkSample>& table, Interpolation interpolation) {
    assert(table.front().lambdaNm <= cie::kLambdaMinNm && table.back().lambdaNm >= cie::kLambdaMaxNm);
    const std::vector<std::complex<double>> eta = sampleIor(table, interpolation);
    Fit fit{};
    fit.reflectivity = projectSpectrum(eta, [](const std::complex<double>& e) { return referenceConductorFresnelAt(e, 1.0); });
    fit.averageTarget = projectSpectrum(eta, [](const std::complex<double>& e) {
        return cosineAverageFresnel([&](double mu) { return referenceConductorFresnelAt(e, mu); });
    });
    for (int c = 0; c < 3; ++c) {
        const double r = fit.reflectivity[c];
        const double target = fit.averageTarget[c];
        if (!(r > kMinReflectivity && r < kMaxReflectivity)) {
            std::cerr << "fitGulbrandsen: channel " << c << " reflectivity " << r << " is outside ("
                      << kMinReflectivity << ", " << kMaxReflectivity << ")\n";
            return std::nullopt;
        }
        const double lowest = modelAverage(r, 0.0);
        const double highest = modelAverage(r, 1.0);
        if (!(target >= lowest && target <= highest)) {
            std::cerr << "fitGulbrandsen: channel " << c << " average " << target << " is outside the edgeTint range ["
                      << lowest << ", " << highest << "]\n";
            return std::nullopt;
        }
        fit.edgeTint[c] = solveEdgeTint(r, target);
    }
    for (int j = 0; j < kResidualAngles; ++j) {
        const double mu = static_cast<double>(j) / (kResidualAngles - 1);
        const glm::dvec3 measured =
            projectSpectrum(eta, [mu](const std::complex<double>& e) { return referenceConductorFresnelAt(e, mu); });
        for (int c = 0; c < 3; ++c) {
            const double model = referenceConductorFresnel(fit.reflectivity[c], fit.edgeTint[c], mu);
            fit.angularResidual[c] = std::max(fit.angularResidual[c], std::abs(model - measured[c]));
        }
    }
    return fit;
}

}  // namespace tools::metal_fit
