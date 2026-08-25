#pragma once

#include <optional>
#include <string>

namespace engine::gfx {

// Owns one linked GL program object, move-only. Compile/link failure is surfaced via std::optional rather than std::exit: a bad shader during active development is a common, recoverable-at-the-call-site failure, unlike Window's/HdrFramebuffer's internal-configuration failures, which have no meaningful recovery path.
class ShaderProgram {
public:
    ~ShaderProgram();

    ShaderProgram(const ShaderProgram&) = delete;
    ShaderProgram& operator=(const ShaderProgram&) = delete;
    ShaderProgram(ShaderProgram&& other) noexcept;
    ShaderProgram& operator=(ShaderProgram&& other) noexcept;

    // Reads both files, then delegates to loadFromSource — the two entry points share one compile/link/error-check implementation.
    static std::optional<ShaderProgram> loadFromFiles(const std::string& vertPath,
                                                        const std::string& fragPath);

    // First-class entry point, not just loadFromFiles's implementation detail: Stage F compiles OCIO's runtime-generated GLSL text, which never exists as a file on disk.
    static std::optional<ShaderProgram> loadFromSource(const std::string& vertSrc,
                                                        const std::string& fragSrc);

    void use() const;

    // -1 (GL's own sentinel) if name doesn't match an active uniform. Not cached: only a couple of uniforms exist this stage, each looked up once at startup — a cache would solve a cost that doesn't exist yet.
    [[nodiscard]] int uniformLocation(const std::string& name) const;

private:
    explicit ShaderProgram(unsigned int program);

    unsigned int program_ = 0;
};

}  // namespace engine::gfx
