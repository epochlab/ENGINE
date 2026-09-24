#pragma once

#include "pathtracer/debug/aov.h"
#include "pathtracer/gfx/hdr_image.h"
#include "pathtracer/scene/path_tracer.h"
#include "pathtracer/scene/rasterizer.h"

namespace pathtracer::debug {

// AovId to the buffer lane holding it, for the two producers that keep their AOVs in a struct of named HdrImages.
// Separate from aov.h so that header stays free of Embree and OpenEXR. Member pointers rather than references so
// both are total functions: null for an AovId the producer does not own, so a caller resolves once and branches.
using PathTracedLane = pathtracer::gfx::HdrImage pathtracer::scene::PathTraceResult::*;
using GBufferLane = pathtracer::gfx::HdrImage pathtracer::scene::RasterGBuffer::*;

// Non-null exactly when aovSource(aov) == AovSource::PathTraced.
[[nodiscard]] PathTracedLane pathTracedLane(AovId aov);

// Non-null exactly when aovSource(aov) == AovSource::GBuffer.
[[nodiscard]] GBufferLane gbufferLane(AovId aov);

}  // namespace pathtracer::debug
