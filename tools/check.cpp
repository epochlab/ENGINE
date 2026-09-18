#include "check.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <thread>

#include "stats.h"

namespace tools::check {
namespace {

// SplitMix64 (Steele, Lea & Flood, OOPSLA 2014): a full-avalanche finalizer, so a small structured input produces a
// well-separated seed without needing a warm-up.
std::uint64_t splitmix64(std::uint64_t x) {
    x += 0x9E3779B97F4A7C15ULL;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
    return x ^ (x >> 31);
}

// FNV-1a: only needs to map a name to a well-spread 64-bit value; SplitMix64 above supplies the avalanche.
std::uint64_t fnv1a64(std::string_view text) {
    std::uint64_t hash = 0xCBF29CE484222325ULL;
    for (const char c : text) {
        hash = (hash ^ static_cast<unsigned char>(c)) * 0x100000001B3ULL;
    }
    return hash;
}

std::vector<Registration>& mutableRegistry() {
    static std::vector<Registration> checks;
    return checks;
}

const char* speedName(Speed speed) { return speed == Speed::Fast ? "fast" : "slow"; }
const char* kindName(Kind kind) { return kind == Kind::Exact ? "exact" : "statistical"; }

// An order of magnitude above the measured worst case in each class: a hang detector, never a performance assertion.
int timeoutSeconds(Speed speed) { return speed == Speed::Fast ? 60 : 900; }

}  // namespace

const std::vector<Registration>& registry() { return mutableRegistry(); }

void registerCheck(const Registration& registration) { mutableRegistry().push_back(registration); }

Context::Context(const Registration& registration, std::uint64_t suiteSeed, std::string_view suiteName, int threads)
    : seed_(splitmix64(suiteSeed ^ fnv1a64(suiteName) ^ fnv1a64(registration.name))), threads_(threads) {}

std::uint64_t Context::subSeed(std::string_view label) const { return splitmix64(seed_ ^ fnv1a64(label)); }

void Context::plan(int assertions) { planned_ = assertions; }

double Context::alpha() const {
    // Corrected across ALL registered checks, not the filtered subset: otherwise `--check=x` would run at a different
    // significance than the full suite and a check's verdict would depend on how it was invoked.
    const double perCheck = stats::sidak(kFamilyAlpha, static_cast<int>(registry().size()));
    return stats::sidak(perCheck, planned_ > 0 ? planned_ : 1);
}

void Context::recordFailure(std::string_view expression, const std::string& detail, const char* file, int line) {
    ++failures_;
    ++asserted_;
    std::cout << "    FAIL " << file << ":" << line << "  " << expression << "\n";
    if (!detail.empty()) {
        std::cout << "         " << detail << "\n";
    }
}

int run(int argc, char** argv, const char* suiteName) {
    std::uint64_t suiteSeed = 0x243F6A8885A308D3ULL;  // fixed: determinism is a requirement, so never clock-derived
    std::string exactName;
    std::string filter;
    // POST_BUILD discovery runs the binary by its full $<TARGET_FILE> path, so argv[0] is what ctest must invoke.
    const std::string exePath = argv[0] != nullptr ? argv[0] : suiteName;
    int threads = std::max(1, static_cast<int>(std::thread::hardware_concurrency()));
    bool list = false;
    bool listCtest = false;

    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--list") {
            list = true;
        } else if (arg == "--list-ctest") {
            listCtest = true;
        } else if (arg.starts_with("--check=")) {
            exactName = arg.substr(std::strlen("--check="));
        } else if (arg.starts_with("--filter=")) {
            filter = arg.substr(std::strlen("--filter="));
        } else if (arg.starts_with("--seed=")) {
            suiteSeed = std::strtoull(std::string(arg.substr(std::strlen("--seed="))).c_str(), nullptr, 10);
        } else if (arg.starts_with("--threads=")) {
            threads = std::max(1, std::atoi(std::string(arg.substr(std::strlen("--threads="))).c_str()));
        } else {
            std::cerr << suiteName << ": unrecognised argument " << arg
                      << "\nusage: " << suiteName
                      << " [--list] [--list-ctest] [--check=NAME] [--filter=SUBSTRING] [--seed=N] [--threads=N]\n";
            return EXIT_FAILURE;
        }
    }

    if (list) {
        for (const Registration& check : registry()) {
            std::cout << check.name << "  " << speedName(check.speed) << "  " << kindName(check.kind) << "\n";
        }
        return EXIT_SUCCESS;
    }

    // Emits the ctest fragment the build consumes through TEST_INCLUDE_FILES, so adding a check adds a test with no
    // CMake edit. Same mechanism CMake's own gtest_discover_tests uses in POST_BUILD mode.
    if (listCtest) {
        for (const Registration& check : registry()) {
            const int perTest = check.speed == Speed::Slow ? threads : 1;
            // Old-style positional add_test: the NAME/COMMAND signature exists only at configure time, not in the
            // script ctest include files are read as.
            std::cout << "add_test(" << suiteName << "." << check.name << " \"" << exePath << "\" --check="
                      << check.name << " --seed=" << suiteSeed << " --threads=" << perTest << ")\n"
                      << "set_tests_properties(" << suiteName << "." << check.name << " PROPERTIES LABELS \""
                      << suiteName << ";" << speedName(check.speed) << ";" << kindName(check.kind) << "\" TIMEOUT "
                      << timeoutSeconds(check.speed) << " PROCESSORS " << perTest << ")\n";
        }
        return EXIT_SUCCESS;
    }

    int failures = 0;
    int ran = 0;
    const auto suiteStart = std::chrono::steady_clock::now();
    for (const Registration& check : registry()) {
        if (!exactName.empty() && exactName != check.name) {
            continue;
        }
        if (!filter.empty() && std::string_view(check.name).find(filter) == std::string_view::npos) {
            continue;
        }
        ++ran;
        Context ctx(check, suiteSeed, suiteName, threads);
        const auto start = std::chrono::steady_clock::now();
        check.fn(ctx);
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

        int checkFailures = ctx.failures();
        // A check whose realized assertion count disagrees with plan() has been corrected for the wrong number of
        // tests, so its bands are wrong even if every one of them passed.
        if (ctx.planned() >= 0 && ctx.asserted() != ctx.planned()) {
            std::cout << "    FAIL " << check.name << " planned " << ctx.planned() << " assertions but made "
                      << ctx.asserted() << "\n";
            ++checkFailures;
        }
        failures += checkFailures;
        std::printf("  [%7.3fs] %-46s %s  (%d assertions)\n", seconds, check.name,
                    checkFailures == 0 ? "PASS" : "FAIL", ctx.asserted());
    }

    const double total = std::chrono::duration<double>(std::chrono::steady_clock::now() - suiteStart).count();
    if (ran == 0) {
        std::cerr << suiteName << ": no check matched\n";
        return EXIT_FAILURE;
    }
    std::printf("\n%s: %d check%s in %.3fs -- %s\n", suiteName, ran, ran == 1 ? "" : "s", total,
                failures == 0 ? "all passed" : "FAILURES PRESENT");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}

}  // namespace tools::check
