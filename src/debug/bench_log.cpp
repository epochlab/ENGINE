#include "pathtracer/debug/bench_log.h"

#include <fcntl.h>
#include <sys/resource.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <ctime>
#include <iostream>

#include <embree4/rtcore_config.h>
#include <zlib.h>

#include "pathtracer/debug/memory_tracker.h"
#include "pathtracer/debug/system_info.h"

namespace pathtracer::debug {

namespace {

std::string utcTimestamp() {
    const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm utc{};
    gmtime_r(&now, &utc);
    std::array<char, 32> text{};
    std::strftime(text.data(), text.size(), "%Y-%m-%dT%H:%M:%SZ", &utc);
    return text.data();
}

nlohmann::json buildJson() {
    const BuildInfo build = buildInfo();
    return {{"git", build.gitSha},       {"uuid", executableUuid()}, {"type", build.buildType},
            {"compiler", build.compiler}, {"march", build.march},     {"ipo", build.ipo},
            {"embree", RTC_VERSION_STRING}};
}

nlohmann::json hostJson() {
    const HostInfo host = queryHostInfo();
    return {{"logical_cpus", host.logicalCpuTotal},
            {"perf0", {{"name", host.perfLevel0Name}, {"cpus", host.perfLevel0LogicalCpu}, {"l2_bytes", host.perfLevel0L2Bytes}}},
            {"perf1", {{"name", host.perfLevel1Name}, {"cpus", host.perfLevel1LogicalCpu}, {"l2_bytes", host.perfLevel1L2Bytes}}},
            {"cache_line_bytes", host.cacheLineBytes},
            {"os", host.osProductVersion},
            {"os_build", host.osBuild},
            {"ram_bytes", totalSystemBytes()}};
}

// Whole-process CPU time and scheduler interference: user_s sums every thread, so it measures work independent of how the pool was scheduled; nivcsw counts preemptions, the signature of a disturbed run.
nlohmann::json rusageJson() {
    rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    const auto seconds = [](const timeval& t) { return static_cast<double>(t.tv_sec) + (static_cast<double>(t.tv_usec) * 1e-6); };
    return {{"user_s", seconds(usage.ru_utime)},
            {"sys_s", seconds(usage.ru_stime)},
            {"nvcsw", usage.ru_nvcsw},
            {"nivcsw", usage.ru_nivcsw},
            {"maxrss_bytes", usage.ru_maxrss}};  // Darwin reports bytes
}

}  // namespace

bool appendBenchRecord(const std::string& path, const BenchRecord& record) {
    const nlohmann::json line = {{"schema", kBenchLogSchema}, {"tool", record.tool},     {"time_utc", utcTimestamp()},
                                 {"pid", getpid()},            {"build", buildJson()},   {"host", hostJson()},
                                 {"argv", record.argv},        {"config", record.config}, {"samples", record.samples},
                                 {"rusage", rusageJson()},     {"work", record.work}};
    const std::string text = line.dump() + '\n';
    const int fd = open(path.c_str(), O_WRONLY | O_APPEND | O_CREAT | O_CLOEXEC, 0644);
    if (fd < 0) {
        std::cerr << "bench_log: cannot open " << path << ": " << std::strerror(errno) << '\n';
        return false;
    }
    // One write, never a retry loop: a resumed partial write would no longer be a single atomic append.
    const ssize_t written = write(fd, text.data(), text.size());
    if (written != static_cast<ssize_t>(text.size())) {
        std::cerr << "bench_log: short or failed write to " << path << ": " << std::strerror(errno) << '\n';
        close(fd);
        return false;
    }
    if (close(fd) != 0) {
        std::cerr << "bench_log: close failed on " << path << ": " << std::strerror(errno) << '\n';
        return false;
    }
    return true;
}

std::uint32_t floatCrc32(std::span<const float> values) {
    const auto* bytes = reinterpret_cast<const Bytef*>(values.data());
    uLong crc = crc32(0L, Z_NULL, 0);
    // zlib takes uInt lengths; feed in chunks so buffers past 4 GiB stay correct.
    std::size_t remaining = values.size_bytes();
    while (remaining > 0) {
        const auto chunk = static_cast<uInt>(std::min<std::size_t>(remaining, 1U << 30U));
        crc = crc32(crc, bytes, chunk);
        bytes += chunk;
        remaining -= chunk;
    }
    return static_cast<std::uint32_t>(crc);
}

}  // namespace pathtracer::debug
