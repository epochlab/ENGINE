#pragma once

#include "engine/debug/aov.h"
#include "engine/gfx/hdr_image.h"
#include "engine/scene/path_tracer.h"
#include "engine/scene/rasterizer.h"

namespace engine::debug {

// AovId to the buffer lane that holds it, for the two producers that keep their AOVs in a struct of named HdrImages.
// Separate from aov.h so that header stays free of Embree and OpenEXR: main.cpp's HUD needs only the enum and the display names, and should not pull in the whole renderer to draw a combo box.
// Member pointers rather than references so both are total functions -- null for an AovId the producer does not own, which is what lets a caller resolve a lane once and reuse it across passes.
using PathTracedLane = engine::gfx::HdrImage engine::scene::PathTraceResult::*;
using GBufferLane = engine::gfx::HdrImage engine::scene::RasterGBuffer::*;

// Non-null exactly when aovSource(aov) == AovSource::PathTraced.
[[nodiscard]] PathTracedLane pathTracedLane(AovId aov);

// Non-null exactly when aovSource(aov) == AovSource::GBuffer.
[[nodiscard]] GBufferLane gbufferLane(AovId aov);

}  // namespace engine::debug
