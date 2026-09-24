#pragma once

#include <optional>
#include <string>

namespace pathtracer::gfx {

// Owns one linked GL program object, move-only. Compile and link failure is surfaced through std::optional rather
// than std::exit: a bad shader during development is recoverable, and the caller decides.
class ShaderProgram {
public:
    ~ShaderProgram();

    ShaderProgram(const ShaderProgram&) = delete;
    ShaderProgram& operator=(const ShaderProgram&) = delete;
    ShaderProgram(ShaderProgram&& other) noexcept;
    ShaderProgram& operator=(ShaderProgram&& other) noexcept;

    // Reads both files, then delegates to loadFromSource: the two entry points share one compile/link/error-check implementation.
    static std::optional<ShaderProgram> loadFromFiles(const std::string& vertPath,
                                                        const std::string& fragPath);

    // First-class entry point, not loadFromFiles's implementation detail: OCIO's runtime-generated GLSL never exists
    // as a file, so it must compile from a string.
    static std::optional<ShaderProgram> loadFromSource(const std::string& vertSrc,
                                                        const std::string& fragSrc);

    void use() const;

    // -1, GL's own sentinel, if name matches no active uniform. Not cached: each is looked up once at startup.
    [[nodiscard]] int uniformLocation(const std::string& name) const;

private:
    explicit ShaderProgram(unsigned int program);

    unsigned int program_ = 0;
};

}  // namespace pathtracer::gfx
