#include "engine/gfx/ocio_cpu_transform.h"

#include <OpenColorIO/OpenColorIO.h>

#include "engine/gfx/ocio_display_transform.h"

namespace OCIO = OCIO_NAMESPACE;

namespace engine::gfx {

void applyOcioDisplayTransform(std::vector<float>& rgb, int width, int height) {
    const OCIO::ConstConfigRcPtr config = OCIO::Config::CreateFromBuiltinConfig(kOcioConfigName);
    const OCIO::ConstProcessorRcPtr processor =
        config->getProcessor(kOcioSceneColorSpace, kOcioSrgbDisplay, kOcioView, OCIO::TRANSFORM_DIR_FORWARD);
    OCIO::PackedImageDesc desc(rgb.data(), width, height, OCIO::CHANNEL_ORDERING_RGB);
    processor->getDefaultCPUProcessor()->apply(desc);
}

}  // namespace engine::gfx
