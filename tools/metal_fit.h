#pragma once

#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

// Measured conductor (lambda, n, k) to Gulbrandsen 2014 (reflectivity, edgeTint) per linear Rec.709 channel, by CIE 1931 integration of the spectral Fresnel reflectance.
namespace tools::metal_fit {

struct NkSample {
    double lambdaNm;
    double n;
    double k;
};

// Rows "lambda_um,n,k" ('#' starts a comment); rejects a table that is unsorted, unphysical (n <= 0 or k < 0) or does not span the CIE observer's 360-830 nm, since extrapolating measured optical constants is not measurement.
[[nodiscard]] std::optional<std::vector<NkSample>> loadNkTable(const std::string& path);

// Johnson & Christy tabulate at uniform photon energy, so both linear-in-wavelength and linear-in-energy are defensible; the fit uses Wavelength and reports the other as a sensitivity.
enum class Interpolation { Wavelength, PhotonEnergy };

struct Fit {
    glm::dvec3 reflectivity;       // r = CIE projection of R(mu = 1, lambda): exact, since Gulbrandsen's R(mu = 1) = r identically
    glm::dvec3 edgeTint;           // g solving E(r, g) = averageTarget, unique since dR/dg >= 0 at every angle (Gulbrandsen eq 13)
    glm::dvec3 averageTarget;      // CIE projection of the spectral cosine-weighted average 2 int R(mu, lambda) mu dmu
    glm::dvec3 angularResidual;    // max over mu of |R_fit(mu) - CIE projection of R(mu, lambda)|: the two-parameter model's shape error
};

// nullopt (with the reason on stderr) when a channel's targets fall outside the Gulbrandsen family, which a clamp would silently misreport.
[[nodiscard]] std::optional<Fit> fitGulbrandsen(const std::vector<NkSample>& table, Interpolation interpolation);

}  // namespace tools::metal_fit
