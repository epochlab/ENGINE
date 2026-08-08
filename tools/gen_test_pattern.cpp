#include <OpenEXR/ImfRgbaFile.h>

#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {

constexpr int kPatchWidth = 100;
constexpr int kPatchCount = 7;
constexpr int kWidth = kPatchWidth * kPatchCount;  // 700
constexpr int kHeight = 100;
constexpr float kGrey18 = 0.18F;

struct Rgb {
    float r;
    float g;
    float b;
};

// Column-uniform by design: nothing in this codebase yet reconciles EXR's
// row-0-top convention with OpenGL's texture-v-origin convention, so a
// pattern identical across every row is immune to an undetected vertical
// flip on the read side — a real open question, correctly not solved
// here.
Rgb colorForColumn(int x) {
    const int patch = x / kPatchWidth;
    switch (patch) {
        case 0:
            return {0.0F, 0.0F, 0.0F};  // black
        case 1:
            return {kGrey18, kGrey18, kGrey18};  // 18% grey
        case 2:
            return {1.0F, 1.0F, 1.0F};  // white
        case 3:
            return {1.0F, 0.0F, 0.0F};  // red
        case 4:
            return {0.0F, 1.0F, 0.0F};  // green
        case 5:
            return {0.0F, 0.0F, 1.0F};  // blue
        default: {
            const int local = x - patch * kPatchWidth;
            const float t = static_cast<float>(local) / static_cast<float>(kPatchWidth - 1);
            return {t, t, t};  // grayscale ramp, exact 0..1 endpoints
        }
    }
}

}  // namespace

int main() {
    const std::string outputPath = ASSET_ROOT_DIR "/textures/test_pattern.exr";

    std::vector<Imf::Rgba> pixels(static_cast<std::size_t>(kWidth) *
                                   static_cast<std::size_t>(kHeight));
    for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
            const Rgb c = colorForColumn(x);
            pixels[static_cast<std::size_t>(y) * kWidth + x] = Imf::Rgba(c.r, c.g, c.b);
        }
    }

    try {
        // WRITE_RGB, not WRITE_RGBA: deliberately omitting alpha so this
        // file exercises the loader's missing-alpha-defaults-to-1.0 path.
        Imf::RgbaOutputFile file(outputPath.c_str(), kWidth, kHeight, Imf::WRITE_RGB);
        file.setFrameBuffer(pixels.data(), 1, kWidth);
        file.writePixels(kHeight);
    } catch (const std::exception& e) {
        std::cerr << "gen_test_pattern: failed to write " << outputPath << ": " << e.what()
                  << '\n';
        return EXIT_FAILURE;
    }

    std::cout << "gen_test_pattern: wrote " << outputPath << " (" << kWidth << "x" << kHeight
               << ")\n";
    return EXIT_SUCCESS;
}
