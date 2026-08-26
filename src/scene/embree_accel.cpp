#include "engine/scene/embree_accel.h"

#include <cstring>
#include <iostream>
#include <utility>

#include <embree4/rtcore.h>

namespace engine::scene {

namespace {

void logEmbreeError(void* /*userPtr*/, RTCError code, const char* str) {
    std::cerr << "EmbreeAccel: " << rtcGetErrorString(code) << ": " << str << "\n";
}

}  // namespace

EmbreeAccel::EmbreeAccel(RTCDeviceTy* device, RTCSceneTy* scene, std::vector<Triangle> triangles)
    : device_(device), scene_(scene), triangles_(std::move(triangles)),
      triangleCount_(static_cast<int>(triangles_.size())) {}

EmbreeAccel::EmbreeAccel(EmbreeAccel&& other) noexcept
    : device_(std::exchange(other.device_, nullptr)),
      scene_(std::exchange(other.scene_, nullptr)),
      triangles_(std::move(other.triangles_)),
      triangleCount_(std::exchange(other.triangleCount_, 0)) {}

EmbreeAccel& EmbreeAccel::operator=(EmbreeAccel&& other) noexcept {
    if (this != &other) {
        release();
        device_ = std::exchange(other.device_, nullptr);
        scene_ = std::exchange(other.scene_, nullptr);
        triangles_ = std::move(other.triangles_);
        triangleCount_ = std::exchange(other.triangleCount_, 0);
    }
    return *this;
}

EmbreeAccel::~EmbreeAccel() { release(); }

void EmbreeAccel::release() {
    if (scene_ != nullptr) {
        rtcReleaseScene(scene_);
        scene_ = nullptr;
    }
    if (device_ != nullptr) {
        rtcReleaseDevice(device_);
        device_ = nullptr;
    }
}

std::optional<EmbreeAccel> EmbreeAccel::build(std::vector<Triangle> triangles) {
    RTCDevice device = rtcNewDevice(nullptr);
    if (device == nullptr) {
        std::cerr << "EmbreeAccel::build: rtcNewDevice failed\n";
        return std::nullopt;
    }
    rtcSetDeviceErrorFunction(device, logEmbreeError, nullptr);

    RTCScene scene = rtcNewScene(device);
    const int triangleCount = static_cast<int>(triangles.size());

    if (triangleCount > 0) {
        RTCGeometry geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
        rtcSetGeometryBuildQuality(geometry, RTC_BUILD_QUALITY_HIGH);

        // triangles is non-indexed triangle soup (Triangle{v0,v1,v2}, three unique vertices per
        // triangle) -- shared directly as Embree's vertex buffer (stride = sizeof(glm::vec3), no
        // copy) rather than deduplicated into an indexed mesh, matching the data's existing shape.
        static_assert(sizeof(Triangle) == 3 * sizeof(glm::vec3),
                      "Triangle must be tightly packed for the shared vertex buffer stride below");
        // Embree reads the last vertex of a shared buffer with a 16-byte SSE load, so the buffer
        // must have at least one float of padding past the last vertex (Embree's documented shared-
        // buffer contract) -- Triangle has no such slack, so grow triangles' capacity (not size) by
        // one vertex worth of memory before handing its pointer to Embree. Must happen before the
        // pointer below is captured, and triangles must not reallocate (no further push_back/reserve)
        // for the rest of this function or after the std::move into the returned EmbreeAccel.
        triangles.reserve(triangles.size() + 1);
        rtcSetSharedGeometryBuffer(geometry, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3,
                                    triangles.data(), 0, sizeof(glm::vec3),
                                    static_cast<std::size_t>(triangleCount) * 3);

        auto* indices = static_cast<unsigned int*>(rtcSetNewGeometryBuffer(
            geometry, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3 * sizeof(unsigned int),
            static_cast<std::size_t>(triangleCount)));
        for (int i = 0; i < triangleCount; ++i) {
            indices[(3 * i) + 0] = static_cast<unsigned int>((3 * i) + 0);
            indices[(3 * i) + 1] = static_cast<unsigned int>((3 * i) + 1);
            indices[(3 * i) + 2] = static_cast<unsigned int>((3 * i) + 2);
        }

        rtcCommitGeometry(geometry);
        rtcAttachGeometry(scene, geometry);
        rtcReleaseGeometry(geometry);
    }

    rtcCommitScene(scene);

    if (rtcGetDeviceError(device) != RTC_ERROR_NONE) {
        rtcReleaseScene(scene);
        rtcReleaseDevice(device);
        return std::nullopt;
    }

    return EmbreeAccel(device, scene, std::move(triangles));
}

std::optional<Hit> EmbreeAccel::intersect(const Ray& ray) const {
    RTCRayHit rayHit{};
    rayHit.ray.org_x = ray.origin.x;
    rayHit.ray.org_y = ray.origin.y;
    rayHit.ray.org_z = ray.origin.z;
    rayHit.ray.dir_x = ray.dir.x;
    rayHit.ray.dir_y = ray.dir.y;
    rayHit.ray.dir_z = ray.dir.z;
    rayHit.ray.tnear = ray.tMin;
    rayHit.ray.tfar = ray.tMax;
    rayHit.ray.mask = 0xFFFFFFFFU;
    rayHit.ray.flags = 0;
    rayHit.hit.geomID = RTC_INVALID_GEOMETRY_ID;

    rtcIntersect1(scene_, &rayHit, nullptr);

    if (rayHit.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
        return std::nullopt;
    }
    return Hit{rayHit.ray.tfar, static_cast<int>(rayHit.hit.primID), rayHit.hit.u, rayHit.hit.v};
}

AabbBounds EmbreeAccel::sceneBounds() const {
    RTCBounds bounds{};
    rtcGetSceneBounds(scene_, &bounds);
    return AabbBounds{glm::vec3(bounds.lower_x, bounds.lower_y, bounds.lower_z),
                       glm::vec3(bounds.upper_x, bounds.upper_y, bounds.upper_z)};
}

bool EmbreeAccel::occluded(const Ray& ray) const {
    RTCRay embreeRay{};
    embreeRay.org_x = ray.origin.x;
    embreeRay.org_y = ray.origin.y;
    embreeRay.org_z = ray.origin.z;
    embreeRay.dir_x = ray.dir.x;
    embreeRay.dir_y = ray.dir.y;
    embreeRay.dir_z = ray.dir.z;
    embreeRay.tnear = ray.tMin;
    embreeRay.tfar = ray.tMax;
    embreeRay.mask = 0xFFFFFFFFU;
    embreeRay.flags = 0;

    rtcOccluded1(scene_, &embreeRay, nullptr);

    // rtcOccluded1 signals a hit by setting tfar to -inf, per Embree convention -- it doesn't
    // populate a hit record (there is none to check) since occlusion is a boolean query.
    return embreeRay.tfar < 0.0F;
}

}  // namespace engine::scene
