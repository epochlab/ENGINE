#include "pathtracer/gfx/ocio_cpu_transform.h"

#include <OpenColorIO/OpenColorIO.h>

#include "pathtracer/gfx/ocio_display_transform.h"

namespace OCIO = OCIO_NAMESPACE;

namespace pathtracer::gfx {

void applyOcioDisplayTransform(std::vector<float>& rgb, int width, int height) {
    const OCIO::ConstConfigRcPtr config = OCIO::Config::CreateFromBuiltinConfig(kOcioConfigName);
    const OCIO::ConstProcessorRcPtr processor =
        config->getProcessor(kOcioSceneColorSpace, kOcioSrgbDisplay, kOcioView, OCIO::TRANSFORM_DIR_FORWARD);
    const OCIO::PackedImageDesc desc(rgb.data(), width, height, OCIO::CHANNEL_ORDERING_RGB);
    processor->getDefaultCPUProcessor()->apply(desc);
}

}  // namespace pathtracer::gfx
