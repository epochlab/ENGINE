#pragma once

#include "pathtracer/debug/aov.h"
#include "pathtracer/gfx/hdr_image.h"
#include "pathtracer/scene/path_tracer.h"
#include "pathtracer/scene/rasterizer.h"

namespace pathtracer::debug {

// AovId to the buffer lane holding it. Member pointers, so both are total functions: null for an AovId the producer does not own.
using PathTracedLane = pathtracer::gfx::HdrImage pathtracer::scene::PathTraceResult::*;
using GBufferLane = pathtracer::gfx::HdrImage pathtracer::scene::RasterGBuffer::*;

// Non-null exactly when aovSource(aov) == AovSource::PathTraced.
[[nodiscard]] PathTracedLane pathTracedLane(AovId aov);

// Non-null exactly when aovSource(aov) == AovSource::GBuffer.
[[nodiscard]] GBufferLane gbufferLane(AovId aov);

}  // namespace pathtracer::debug
