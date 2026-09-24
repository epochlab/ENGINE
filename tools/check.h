#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

// Shared runner for the validators. Not a test framework: there are no fixtures, no mocking and no exceptions here,
// because a numeric Monte Carlo suite needs none of them. What it does supply is the three things six standalone
// main()s could not -- a check is addressable by name (so ctest reports which one failed rather than which binary), a
// check's randomness is reproducible from the command line, and the significance level every statistical band is
// derived from lives in exactly one place.
namespace tools::check {

// Fast checks are analytic or combinatorial and run in well under a second; Slow ones carry the Monte Carlo sweeps.
// The distinction is the only input to the ctest label/timeout decision, and is deliberately binary rather than a
// millisecond budget, which would be a performance assertion in disguise.
enum class Speed { Fast, Slow };

// Exact checks assert integer counts, bit-identity or a deterministic forward-error bound: they have no tolerance
// derived from a distribution and therefore cannot flake. Statistical ones carry a confidence band. `ctest -L exact`
// is the subset that cannot flake by construction, which is what a pre-commit gate wants to be.
enum class Kind { Exact, Statistical };

class Context;
using Fn = void (*)(Context&);

struct Registration {
    const char* name;
    Fn fn;
    Speed speed;
    Kind kind;
};

// Function-local static storage, so registration order across translation units cannot matter.
const std::vector<Registration>& registry();
void registerCheck(const Registration& registration);

struct Registrar {
    explicit Registrar(const Registration& registration) { registerCheck(registration); }
};

class Context {
public:
    Context(const Registration& registration, std::uint64_t suiteSeed, std::string_view suiteName, int threads);

    // This check's master seed, derived from the check's NAME rather than its position in the registry: adding or
    // reordering a check must not reshuffle every other check's realization, or a migration's unrelated borderline
    // movements become indistinguishable from real regressions.
    [[nodiscard]] std::uint64_t seed() const { return seed_; }
    // Per-case seed, by the same name-derived argument one level down: adding a row to a sweep leaves the existing
    // rows bit-identical.
    [[nodiscard]] std::uint64_t subSeed(std::string_view label) const;

    [[nodiscard]] int threads() const { return threads_; }

    // Declares how many assertions this check makes, so the per-assertion significance can be corrected for them.
    // Call once, before asserting. The runner fails the check if the realized count disagrees, which is what stops
    // plan() becoming a lie that silently loosens every band below it.
    void plan(int assertions);
    // Per-assertion two-sided significance, already corrected across the whole registry and this check's assertions.
    [[nodiscard]] double alpha() const;

    void recordPass() { ++asserted_; }
    void recordFailure(std::string_view expression, const std::string& detail, const char* file, int line);

    [[nodiscard]] int failures() const { return failures_; }
    [[nodiscard]] int asserted() const { return asserted_; }
    [[nodiscard]] int planned() const { return planned_; }

private:
    std::uint64_t seed_;
    int threads_;
    int planned_ = -1;
    int asserted_ = 0;
    int failures_ = 0;
};

// Family-wise error rate for one validator binary's run. The family is the binary: a binary cannot know the other
// binaries' assertion counts without the build system telling it, and coupling a statistical constant to the target
// list is worse than stating the arithmetic. Across B binaries the suite-wide rate is 1-(1-kFamilyAlpha)^B, about
// 6e-4 at B=6 -- roughly one spurious red per 1700 full-suite runs.
// This is the only free parameter in the statistical design, and it is a stated ERROR RATE, not a tolerance.
inline constexpr double kFamilyAlpha = 1e-4;

int run(int argc, char** argv, const char* suiteName);

}  // namespace tools::check

// Defines and registers a check. The body receives `ctx`.
#define PT_CHECK(checkName, speed, kind)                                                    \
    static void checkName(::tools::check::Context&);                                            \
    static const ::tools::check::Registrar checkName##_registrar{                               \
        {#checkName, &checkName, ::tools::check::Speed::speed, ::tools::check::Kind::kind}};    \
    static void checkName(::tools::check::Context& ctx)

#define PT_CHECK_MAIN(suiteName) \
    int main(int argc, char** argv) { return ::tools::check::run(argc, argv, suiteName); }

// The caller writes the PASS condition and the macro inverts it exactly once. That is what makes NaN fail: NaN
// compares false against every ordered operator, so a negated-pass form rejects it while a "fail if worse than"
// form would silently accept it. Never restate these as failure conditions.
#define PT_EXPECT(ctx, passExpression, detail)                                     \
    do {                                                                               \
        if (!(passExpression)) {                                                       \
            (ctx).recordFailure(#passExpression, (detail), __FILE__, __LINE__);         \
        } else {                                                                       \
            (ctx).recordPass();                                                        \
        }                                                                              \
    } while (false)
