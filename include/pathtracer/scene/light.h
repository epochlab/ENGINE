#pragma once

#include <optional>
#include <vector>

#include <glm/glm.hpp>

#include "pathtracer/scene/environment_map.h"
#include "pathtracer/scene/sampler.h"

namespace pathtracer::scene {

// A rectangular emitter: origin is one corner, edge0/edge1 span the sides. Urena/Fajardo/King's spherical-rectangle
// sampling is exact only for a rectangle, so edge0 must be perpendicular to edge1. Emits from the face whose outward
// normal is normalize(cross(edge0, edge1)); the back face is dark unless twoSided (Arnold quad_light semantics).
struct QuadLight {
    QuadLight(glm::vec3 origin, glm::vec3 edge0, glm::vec3 edge1, glm::vec3 radiance, bool twoSided = false)
        : origin(origin), edge0(edge0), edge1(edge1), radiance(radiance), twoSided(twoSided),
          normal(glm::normalize(glm::cross(edge0, edge1))) {}

    glm::vec3 origin;
    glm::vec3 edge0;
    glm::vec3 edge1;
    glm::vec3 radiance;  // constant Le over the emitting face, colour * intensity
    bool twoSided;
    // Derived from edge0/edge1 once here rather than per query: quadRadianceToward runs on every NEE sample and
    // every emitter hit, and appendQuadLights needs the same vector.
    glm::vec3 normal;
};

// Ureña, Fajardo & King, "An Area-Preserving Parametrization for Spherical Rectangles" (EGSR 2013), as given in
// PBRT 4th ed. 12.5.3: an exact constant-pdf solid-angle sampler, avoiding the variance a uniform-area
// sample-then-reweight pays. buildSphericalRectangle, sample() and pdfQuad all derive the same solid angle.
struct SphericalRectangle {
    glm::vec3 referencePoint;
    glm::vec3 x, y, z;  // local orthonormal frame, z chosen so the reference point has z0 < 0
    float z0;
    float x0, x1, y0, y1;
    float b0, b1;
    float k;
    float solidAngle;  // steradians subtended by the rectangle at referencePoint; pdf = 1/solidAngle

    // u in [0,1)^2 -> a world-space point on the rectangle, uniform in solid angle as seen from referencePoint.
    [[nodiscard]] glm::vec3 sample(glm::vec2 u) const;
};

// nullopt iff the rectangle subtends no solid angle at referencePoint: a degenerate quad (parallel
// edges), or referencePoint exactly in the rectangle's own plane.
[[nodiscard]] std::optional<SphericalRectangle> buildSphericalRectangle(const QuadLight& quad,
                                                                         const glm::vec3& referencePoint);

struct LightSample {
    glm::vec3 direction;  // unit, from the shading point toward the light
    glm::vec3 radiance;   // Le arriving from that direction (0 if the light doesn't emit that way)
    float pdf;            // solid-angle density of `direction`, INCLUDING the light-selection probability; > 0
    float distance;       // Euclidean distance to the sampled point; FLT_MAX for the environment
};

// The set of lights NEE can sample from in one renderPathTraced() pass: the environment map, or none when the HUD
// toggle is off, plus zero or more rectangular emitters. Selection is uniform, and a single-light scene draws no
// variate at all, which keeps its sample sequence bit-identical to a renderer with no selection mechanism.
class LightSet {
public:
    // environment == nullptr excludes it from the set entirely (no NEE, no MIS, no miss radiance at
    // any bounce) -- the HUD's environment-light toggle. envRotationRadians/envExposure are read only
    // when environment != nullptr. quads may be empty; the referenced vector must outlive this LightSet.
    LightSet(const EnvironmentMap* environment, float envRotationRadians, float envExposure,
             const std::vector<QuadLight>& quads);

    [[nodiscard]] int count() const;

    // nullopt iff count() == 0, or the one light selected (a quad) has zero solid angle at p.
    [[nodiscard]] std::optional<LightSample> sample(const glm::vec3& p, Sampler& sampler) const;

    // MIS pdf of a BSDF-sampled ray having reached the environment in direction `dir` -- 0 if the
    // environment is excluded from the set.
    [[nodiscard]] float pdfEnvironment(const glm::vec3& dir) const;

    // MIS pdf of a BSDF-sampled ray from `p` having reached quad `quadIndex` -- the ray is presumed to
    // have actually hit it (an Embree intersection already confirmed this), so no direction check is
    // needed: the spherical-rectangle pdf is constant over the light's whole solid angle.
    [[nodiscard]] float pdfQuad(int quadIndex, const glm::vec3& p) const;

    // Le toward `direction` (unit, pointing from the light toward the viewer's side, i.e. the same
    // sense as a ray's own travel direction into the light) -- 0 on the non-emitting back face unless twoSided.
    [[nodiscard]] glm::vec3 quadRadianceToward(int quadIndex, const glm::vec3& direction) const;

    // Radiance sampled from the environment toward `direction` -- 0 if the environment is excluded.
    // nearest: see EnvironmentMap::sampleDirectionNearest vs sampleDirection.
    [[nodiscard]] glm::vec3 environmentRadiance(const glm::vec3& direction, bool nearest) const;

private:
    const EnvironmentMap* environment_;
    float envRotationRadians_;
    float envExposure_;
    const std::vector<QuadLight>& quads_;
};

}  // namespace pathtracer::scene
