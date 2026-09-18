#pragma once

#include <vector>

// The viewer's display path, evaluated on the CPU. Built from the same config/colorspace/display/view constants the
// display shaders are generated from (ocio_display_transform.h), so this is the SAME transform rather than a second
// definition of it -- a comparison render encoded through a separately-declared curve would drift from the viewer the
// moment either side was repinned.
// Lifted out of tools/render_beauty.cpp, where it was a private helper: keeping it there meant the only CPU-evaluable
// part of the colour pipeline could not be tested, and the pinned config's central property (that "Un-tone-mapped" is
// a pure colorimetric pass) was asserted by a comment rather than by a check. See tools/display_validate.cpp.
namespace engine::gfx {

// In place, RGB triples (no alpha), row-major, width*height*3 elements.
void applyOcioDisplayTransform(std::vector<float>& rgb, int width, int height);

}  // namespace engine::gfx
