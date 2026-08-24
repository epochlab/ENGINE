#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace engine::debug {

// Per-channel (R/G/B) histogram of the currently displayed frame,
// captured every 4th frame via an async GPU->CPU readback — matches
// epochlab/KODAK's approach exactly: downsample the composited frame
// into a small fixed FBO, double-buffer the PBO readback so the render
// thread never stalls waiting on it, and bin whichever buffer was
// written a full capture interval ago each time.
class Histogram {
public:
    static constexpr int kWidth = 256;
    static constexpr int kHeight = 144;
    static constexpr int kBins = 256;
    static constexpr int kCaptureIntervalFrames = 4;

    Histogram();
    ~Histogram();

    Histogram(const Histogram&) = delete;
    Histogram& operator=(const Histogram&) = delete;
    Histogram(Histogram&& other) noexcept;
    Histogram& operator=(Histogram&& other) noexcept;

    // Call once per frame, after the final composited image has been
    // written to the default framebuffer (PostProcessPass::draw) and
    // before the HUD is rendered on top of it. windowWidth/Height are
    // framebuffer pixels.
    void update(int windowWidth, int windowHeight);

    [[nodiscard]] const std::array<std::array<std::uint32_t, kBins>, 3>& bins() const {
        return bins_;
    }

    // False until the first full capture+bin cycle has completed —
    // callers should skip drawing until then.
    [[nodiscard]] bool hasData() const { return hasData_; }

private:
    void destroy();

    unsigned int downsampleFbo_ = 0;
    unsigned int downsampleTexture_ = 0;
    unsigned int pbos_[2] = {0, 0};
    int currentPbo_ = 0;
    int frameCounter_ = 0;
    bool pboWritten_[2] = {false, false};
    bool hasData_ = false;
    std::array<std::array<std::uint32_t, kBins>, 3> bins_{};
    std::size_t byteSize_ = 0;  // reported to engine::debug's GPU memory tracker
};

}  // namespace engine::debug
