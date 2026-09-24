#pragma once

#include <vector>

// The viewer's display path on the CPU, from the same constants as the display shaders: one definition, so a comparison cannot drift.
namespace pathtracer::gfx {

// In place, RGB triples (no alpha), row-major, width*height*3 elements.
void applyOcioDisplayTransform(std::vector<float>& rgb, int width, int height);

}  // namespace pathtracer::gfx
