#pragma once

#include <cmath>
#include <limits>

// Shared statistics for the validators: quadrature, survival functions, quantiles and confidence bands.
namespace tools::stats {

// Composite Simpson's rule over `panels` (even) intervals. Generic in the integrand's return type so it composes with
// itself for nested quadrature -- the outer call's integrand returns the inner call's value, not a scalar.
template <typename Integrand>
auto simpson(double lower, double upper, int panels, Integrand f) -> decltype(f(0.0)) {
    const double h = (upper - lower) / panels;
    auto sum = f(lower) + f(upper);
    for (int i = 1; i < panels; ++i) {
        sum = sum + ((i % 2 == 1 ? 4.0 : 2.0) * f(lower + (i * h)));
    }
    return (h / 3.0) * sum;
}

// Regularized upper incomplete gamma Q(a, x): series below the crossover, continued fraction above (Numerical Recipes
// 3rd ed. 6.2). Supplies the chi-square survival function so a test can state a significance level rather than carry a
// critical-value table indexed by degrees of freedom.
inline double regularizedGammaQ(double a, double x) {
    constexpr int kMaxIterations = 300;
    constexpr double kEpsilon = 1e-14;
    const double logGammaA = std::lgamma(a);
    const double tiny = std::numeric_limits<double>::min();
    if (x < a + 1.0) {
        double term = 1.0 / a;
        double sum = term;
        for (int n = 1; n < kMaxIterations && std::fabs(term) > std::fabs(sum) * kEpsilon; ++n) {
            term *= x / (a + n);
            sum += term;
        }
        return 1.0 - (sum * std::exp(-x + (a * std::log(x)) - logGammaA));
    }
    double b = x + 1.0 - a;
    double c = 1.0 / tiny;
    double d = 1.0 / b;
    double h = d;
    for (int i = 1; i < kMaxIterations; ++i) {
        const double an = -i * (i - a);
        b += 2.0;
        d = (an * d) + b;
        if (std::fabs(d) < tiny) { d = tiny; }
        c = b + (an / c);
        if (std::fabs(c) < tiny) { c = tiny; }
        d = 1.0 / d;
        const double delta = d * c;
        h *= delta;
        if (std::fabs(delta - 1.0) <= kEpsilon) { break; }
    }
    return h * std::exp(-x + (a * std::log(x)) - logGammaA);
}

// Upper-tail probability of the chi-square distribution: P(X > chi2) at `dof` degrees of freedom.
inline double chiSquareUpperTail(double chi2, int dof) {
    return regularizedGammaQ(0.5 * dof, 0.5 * chi2);
}

// Per-test significance holding a family-wise error rate of `familyAlpha` across `tests` independent tests
// (Sidak 1967). Exact under independence; Bonferroni's familyAlpha/tests is the arbitrary-dependence alternative and
// agrees with this to better than one part in 10^4 at the levels used here, so the choice is documentation.
inline double sidak(double familyAlpha, int tests) {
    return 1.0 - std::pow(1.0 - familyAlpha, 1.0 / static_cast<double>(tests));
}


// --- Quantiles --------------------------------------------------------------------------------------------------
// These are what retire "6 sigma" as a written constant. 6 sigma two-sided is p = 1.97e-9, a number nobody chose
// deliberately; a quantile evaluated at a stated error rate is a number that moves correctly when the sample count or
// the number of tests changes.

// Inverse standard normal CDF. Wichura 1988, Algorithm AS 241 (Applied Statistics 37(3)), the PPND16 variant:
// relative accuracy ~1e-16 over the whole range, which is far more than any band here needs, but it is a closed
// rational form with no iteration and no convergence caveat to document.
inline double normalQuantile(double p) {
    const double q = p - 0.5;
    if (std::fabs(q) <= 0.425) {
        const double r = 0.180625 - (q * q);
        return q *
               (((((((2509.0809287301226727 * r + 33430.575583588128105) * r + 67265.770927008700853) * r +
                     45921.953931549871457) * r + 13731.693765509461125) * r + 1971.5909503065514427) * r +
                  133.14166789178437745) * r + 3.387132872796366608) /
               (((((((5226.495278852854561 * r + 28729.085735721942674) * r + 39307.89580009271061) * r +
                     21213.794301586595867) * r + 5394.1960214247511077) * r + 687.1870074920579083) * r +
                  42.313330701600911252) * r + 1.0);
    }
    double r = q < 0.0 ? p : 1.0 - p;
    r = std::sqrt(-std::log(r));
    double value = 0.0;
    if (r <= 5.0) {
        r -= 1.6;
        value = (((((((7.7454501427834140764e-4 * r + 0.0227238449892691845833) * r + 0.24178072517745061177) * r +
                      1.27045825245236838258) * r + 3.64784832476320460504) * r + 5.7694972214606914055) * r +
                   4.6303378461565452959) * r + 1.42343711074968357734) /
                (((((((1.05075007164441684324e-9 * r + 5.475938084995344946e-4) * r + 0.0151986665636164571966) * r +
                      0.14810397642748007459) * r + 0.68976733498510000455) * r + 1.6763848301838038494) * r +
                   2.05319162663775882187) * r + 1.0);
    } else {
        r -= 5.0;
        value = (((((((2.01033439929228813265e-7 * r + 2.71155556874348757815e-5) * r + 0.0012426609473880784386) * r +
                      0.026532189526576123093) * r + 0.29656057182850489123) * r + 1.7848265399172913358) * r +
                   5.4637849111641143699) * r + 6.6579046435011037772) /
                (((((((2.04426310338993978564e-15 * r + 1.4215117583164458887e-7) * r + 1.8463183175100546818e-5) * r +
                      7.868691311456132591e-4) * r + 0.0148753612908506148525) * r + 0.13692988092273580531) * r +
                   0.59983220655588793769) * r + 1.0);
    }
    return q < 0.0 ? -value : value;
}

// Continued fraction for the incomplete beta function (Numerical Recipes 3rd ed. 6.4); backs regularizedBetaI.
inline double betaContinuedFraction(double a, double b, double x) {
    constexpr int kMaxIterations = 300;
    constexpr double kEpsilon = 1e-14;
    const double tiny = std::numeric_limits<double>::min();
    const double qab = a + b;
    const double qap = a + 1.0;
    const double qam = a - 1.0;
    double c = 1.0;
    double d = 1.0 - (qab * x / qap);
    if (std::fabs(d) < tiny) { d = tiny; }
    d = 1.0 / d;
    double h = d;
    for (int m = 1; m <= kMaxIterations; ++m) {
        const int m2 = 2 * m;
        double aa = m * (b - m) * x / ((qam + m2) * (a + m2));
        d = 1.0 + (aa * d);
        if (std::fabs(d) < tiny) { d = tiny; }
        c = 1.0 + (aa / c);
        if (std::fabs(c) < tiny) { c = tiny; }
        d = 1.0 / d;
        h *= d * c;
        aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2));
        d = 1.0 + (aa * d);
        if (std::fabs(d) < tiny) { d = tiny; }
        c = 1.0 + (aa / c);
        if (std::fabs(c) < tiny) { c = tiny; }
        d = 1.0 / d;
        const double delta = d * c;
        h *= delta;
        if (std::fabs(delta - 1.0) <= kEpsilon) { break; }
    }
    return h;
}

// Regularized incomplete beta I_x(a,b) (Numerical Recipes 3rd ed. 6.4).
inline double regularizedBetaI(double a, double b, double x) {
    if (x <= 0.0) { return 0.0; }
    if (x >= 1.0) { return 1.0; }
    const double front =
        std::exp(std::lgamma(a + b) - std::lgamma(a) - std::lgamma(b) + (a * std::log(x)) + (b * std::log1p(-x)));
    return x < (a + 1.0) / (a + b + 2.0) ? front * betaContinuedFraction(a, b, x) / a
                                          : 1.0 - (front * betaContinuedFraction(b, a, 1.0 - x) / b);
}

// Two-sided Student-t quantile: the multiplier t such that P(|T| <= t) = 1 - alpha at `dof` degrees of freedom.
// Student-t rather than a normal quantile because the variance is ESTIMATED, not known. At the replicate counts used
// here (R = 16, so 15 dof) the difference from the normal quantile is a factor of ~1.6 at these alphas -- exactly the
// regime where using the normal would produce a band that is too tight and a check that flakes.
inline double studentTTwoSided(double alpha, double dof) {
    // Monotone in t, so bisection is both correct and immune to the convergence caveats an inverse-series would carry.
    const auto tailProbability = [dof](double t) {
        return regularizedBetaI(0.5 * dof, 0.5, dof / (dof + (t * t)));  // = P(|T| > t)
    };
    double low = 0.0;
    double high = 1.0;
    while (tailProbability(high) > alpha && high < 1e12) { high *= 2.0; }
    for (int i = 0; i < 200; ++i) {
        const double mid = 0.5 * (low + high);
        (tailProbability(mid) > alpha ? low : high) = mid;
    }
    return 0.5 * (low + high);
}

// Replicate count for randomized-QMC estimators. Sampler output is one Owen-scrambled Sobol set, so sqrt(N) error does
// not apply; independent scrambles make replicate means iid regardless (Owen 1997b; L'Ecuyer & Lemieux 2002).
// 16: below ~10 the t-quantile inflates the band, above ~32 extra dof buy nothing; a power of two dividing every count.
inline constexpr int kReplicates = 16;

// --- Estimators and bands ---------------------------------------------------------------------------------------

// A half-open interval stated in PASS form, so a NaN value or a NaN bound fails rather than silently passing.
struct Band {
    double lo = 0.0;
    double hi = 0.0;
    [[nodiscard]] bool contains(double v) const { return v >= lo && v <= hi; }
    [[nodiscard]] double halfWidth() const { return 0.5 * (hi - lo); }
};

// Welford's online variance (Welford 1962, Technometrics 4(3); numerics per Chan, Golub & LeVeque 1983): one pass, and
// it does not form the catastrophic sum-of-squares-minus-square-of-sum difference a naive accumulator would.
class Welford {
public:
    void add(double x) {
        ++count_;
        const double delta = x - mean_;
        mean_ += delta / static_cast<double>(count_);
        m2_ += delta * (x - mean_);
    }
    [[nodiscard]] long long count() const { return count_; }
    [[nodiscard]] double mean() const { return mean_; }
    // Unbiased (n-1) sample variance.
    [[nodiscard]] double sampleVariance() const {
        return count_ > 1 ? m2_ / static_cast<double>(count_ - 1) : 0.0;
    }
    [[nodiscard]] double standardError() const {
        return count_ > 1 ? std::sqrt(sampleVariance() / static_cast<double>(count_)) : 0.0;
    }

private:
    long long count_ = 0;
    double mean_ = 0.0;
    double m2_ = 0.0;
};

// Band for the DIFFERENCE of two independent estimators, centred at zero: the null "both estimate the same quantity".
// Their variances add, which a flat "x% of one side" tolerance gets wrong by modelling one side as exact. Welch 1947
// (Biometrika 34) supplies the effective degrees of freedom when the two variances differ, which they generally do --
// a Russian-roulette render is materially noisier than the same render without it.
inline Band differenceBand(const Welford& a, const Welford& b, double alpha) {
    const double va = a.sampleVariance() / static_cast<double>(a.count());
    const double vb = b.sampleVariance() / static_cast<double>(b.count());
    const double combined = va + vb;
    if (!(combined > 0.0)) {
        return Band{0.0, 0.0};  // both estimators are constant; any difference at all is a real one
    }
    const double dofA = static_cast<double>(a.count() - 1);
    const double dofB = static_cast<double>(b.count() - 1);
    const double dof = (combined * combined) / (((va * va) / dofA) + ((vb * vb) / dofB));
    const double half = studentTTwoSided(alpha, dof) * std::sqrt(combined);
    return Band{-half, half};
}

// Wilson score interval for a binomial proportion (Wilson 1927, JASA 22(158)). NOT Wald: Brown, Cai & DasGupta 2001
// (Statistical Science 16(2)) show Wald's coverage is erratic and its width collapses to zero as p approaches 0 or 1,
// which is precisely the degenerate case an occlusion or hit-fraction check can land in.
inline Band wilsonBand(long long successes, long long trials, double alpha) {
    const double n = static_cast<double>(trials);
    const double z = normalQuantile(1.0 - (0.5 * alpha));
    const double phat = static_cast<double>(successes) / n;
    const double z2 = z * z;
    const double denominator = 1.0 + (z2 / n);
    const double centre = (phat + (z2 / (2.0 * n))) / denominator;
    const double half = (z * std::sqrt((phat * (1.0 - phat) / n) + (z2 / (4.0 * n * n)))) / denominator;
    return Band{centre - half, centre + half};
}

}  // namespace tools::stats
