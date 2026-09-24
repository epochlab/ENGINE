#pragma once

#include <vector>

// The viewer's display path evaluated on the CPU, built from the same config, colorspace, display and view constants
// the display shaders are generated from (ocio_display_transform.h). The same transform, not a second definition:
// a comparison render encoded through a separately-declared curve would drift the moment either side was repinned.
namespace pathtracer::gfx {

// In place, RGB triples (no alpha), row-major, width*height*3 elements.
void applyOcioDisplayTransform(std::vector<float>& rgb, int width, int height);

}  // namespace pathtracer::gfx
