#pragma once

#include "pathtracer/debug/aov.h"
#include "pathtracer/gfx/hdr_image.h"
#include "pathtracer/scene/path_tracer.h"
#include "pathtracer/scene/rasterizer.h"

namespace pathtracer::debug {

// AovId to the buffer lane that holds it, for the two producers that keep their AOVs in a struct of named HdrImages.
// Separate from aov.h so that header stays free of Embree and OpenEXR: main.cpp's HUD needs only the enum and the display names, and should not pull in the whole renderer to draw a combo box.
// Member pointers rather than references so both are total functions -- null for an AovId the producer does not own, which is what lets a caller resolve a lane once and reuse it across passes.
using PathTracedLane = pathtracer::gfx::HdrImage pathtracer::scene::PathTraceResult::*;
using GBufferLane = pathtracer::gfx::HdrImage pathtracer::scene::RasterGBuffer::*;

// Non-null exactly when aovSource(aov) == AovSource::PathTraced.
[[nodiscard]] PathTracedLane pathTracedLane(AovId aov);

// Non-null exactly when aovSource(aov) == AovSource::GBuffer.
[[nodiscard]] GBufferLane gbufferLane(AovId aov);

}  // namespace pathtracer::debug
