#include <OpenEXR/ImfArray.h>
#include <OpenEXR/ImfRgbaFile.h>

#include <algorithm>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <regex>
#include <string>
#include <vector>

namespace {

// Box filter: each output texel averages the source block it covers. One-shot asset-prep tool, not a hot path -- clarity over throughput.
std::vector<Imf::Rgba> boxDownsample(const Imf::Array2D<Imf::Rgba>& src, int srcWidth,
                                      int srcHeight, int dstWidth, int dstHeight) {
    std::vector<Imf::Rgba> out(static_cast<std::size_t>(dstWidth) *
                                static_cast<std::size_t>(dstHeight));
    for (int y = 0; y < dstHeight; ++y) {
        const int y0 = y * srcHeight / dstHeight;
        const int y1 = (y + 1) * srcHeight / dstHeight;
        for (int x = 0; x < dstWidth; ++x) {
            const int x0 = x * srcWidth / dstWidth;
            const int x1 = (x + 1) * srcWidth / dstWidth;

            float r = 0.0F;
            float g = 0.0F;
            float b = 0.0F;
            int count = 0;
            for (int sy = y0; sy < y1; ++sy) {
                for (int sx = x0; sx < x1; ++sx) {
                    const Imf::Rgba& texel = src[sy][sx];
                    r += static_cast<float>(texel.r);
                    g += static_cast<float>(texel.g);
                    b += static_cast<float>(texel.b);
                    ++count;
                }
            }
            out[static_cast<std::size_t>(y) * static_cast<std::size_t>(dstWidth) +
                static_cast<std::size_t>(x)] = Imf::Rgba(r / count, g / count, b / count);
        }
    }
    return out;
}

// "2048" -> "2K"; falls back to "<n>px" for a non-1024-aligned size.
std::string sizeLabel(int px) {
    if (px >= 1024 && px % 1024 == 0) {
        return std::to_string(px / 1024) + "K";
    }
    return std::to_string(px) + "px";
}

// Sibling directory named "<source-dir>_<label, lowercased>/", so each resolution's full texture set stays together and the source is untouched. Within that directory, the filename keeps the source name but with its "_<n>K_" resolution token replaced by the new label (or appended before the extension, if no such token is found).
std::filesystem::path outputPathFor(const std::filesystem::path& inputPath,
                                     const std::string& label) {
    static const std::regex kResolutionToken(R"(_[0-9]+[Kk]_)");
    std::string filename = inputPath.filename().string();
    if (std::smatch match; std::regex_search(filename, match, kResolutionToken)) {
        filename = std::regex_replace(filename, kResolutionToken, "_" + label + "_",
                                       std::regex_constants::format_first_only);
    } else {
        filename = inputPath.stem().string() + "_" + label + inputPath.extension().string();
    }

    std::string labelLower = label;
    std::transform(labelLower.begin(), labelLower.end(), labelLower.begin(),
                    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    const std::filesystem::path sourceDir = inputPath.parent_path();
    const std::filesystem::path siblingDir =
        sourceDir.parent_path() / (sourceDir.filename().string() + "_" + labelLower);
    std::filesystem::create_directories(siblingDir);
    return siblingDir / filename;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "usage: downsample <input.exr> <target_long_edge_px>\n";
        return EXIT_FAILURE;
    }
    const std::filesystem::path inputPath = argv[1];
    const int target = std::atoi(argv[2]);
    if (target <= 0) {
        std::cerr << "downsample: target_long_edge_px must be positive\n";
        return EXIT_FAILURE;
    }

    try {
        // Single-channel (R-only) sources -- Roughness/Bump/AO in this project's assets -- read with G/B as 0 (RgbaInputFile's own documented default for a missing channel), so their greyscale value survives the box filter in R only. Consumers already expect this: same convention Texture::createFromExr relies on.
        Imf::RgbaInputFile file(inputPath.c_str());
        const auto& dw = file.dataWindow();
        if (dw.isEmpty()) {
            std::cerr << "downsample: empty data window in " << inputPath.string() << '\n';
            return EXIT_FAILURE;
        }
        const int srcWidth = dw.max.x - dw.min.x + 1;
        const int srcHeight = dw.max.y - dw.min.y + 1;

        Imf::Array2D<Imf::Rgba> pixels;
        pixels.resizeErase(srcHeight, srcWidth);
        file.setFrameBuffer(&pixels[0][0] - dw.min.x - dw.min.y * srcWidth, 1, srcWidth);
        file.readPixels(dw.min.y, dw.max.y);

        const int longEdge = srcWidth > srcHeight ? srcWidth : srcHeight;
        if (target > longEdge) {
            std::cerr << "downsample: target (" << target << "px) exceeds source long edge ("
                       << longEdge << "px) -- this tool downsamples only\n";
            return EXIT_FAILURE;
        }
        const int dstWidth = srcWidth * target / longEdge;
        const int dstHeight = srcHeight * target / longEdge;

        const std::vector<Imf::Rgba> downsampled =
            boxDownsample(pixels, srcWidth, srcHeight, dstWidth, dstHeight);

        const std::filesystem::path outputPath = outputPathFor(inputPath, sizeLabel(target));
        Imf::RgbaOutputFile out(outputPath.c_str(), dstWidth, dstHeight, Imf::WRITE_RGB);
        out.setFrameBuffer(downsampled.data(), 1, dstWidth);
        out.writePixels(dstHeight);

        std::cout << "downsample: wrote " << outputPath.string() << " (" << dstWidth << "x"
                   << dstHeight << ", from " << srcWidth << "x" << srcHeight << ")\n";
    } catch (const std::exception& e) {
        std::cerr << "downsample: failed on " << inputPath.string() << ": " << e.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
