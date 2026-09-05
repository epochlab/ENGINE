#pragma once

#include <optional>
#include <vector>

#include <glm/glm.hpp>

#include "engine/scene/environment_map.h"
#include "engine/scene/sampler.h"

namespace engine::scene {

// A rectangular emitter: origin is one corner, edge0/edge1 span the two sides. Urena/Fajardo/King's spherical-rectangle sampling below is exact only for a rectangle, so edge0 must be perpendicular to edge1 -- enforced for authored scenes by loadSceneConfig (scene_config.h), assumed for lights built directly in the validators.
// Emits uniformly from the face whose outward normal is normalize(cross(edge0, edge1)); the back face emits nothing unless twoSided (Arnold quad_light semantics).
struct QuadLight {
    QuadLight(glm::vec3 origin, glm::vec3 edge0, glm::vec3 edge1, glm::vec3 radiance, bool twoSided = false)
        : origin(origin), edge0(edge0), edge1(edge1), radiance(radiance), twoSided(twoSided),
          normal(glm::normalize(glm::cross(edge0, edge1))) {}

    glm::vec3 origin;
    glm::vec3 edge0;
    glm::vec3 edge1;
    glm::vec3 radiance;  // constant Le over the emitting face, colour * intensity
    bool twoSided;
    // Derived from edge0/edge1 once here rather than per query: quadRadianceToward runs on every NEE sample and every emitter hit, and appendQuadLights needs the same vector again when it injects the light's triangles. Production light types precompute their frame at build for the same reason.
    glm::vec3 normal;
};

// Ureña, Fajardo & King, "An Area-Preserving Parametrization for Spherical Rectangles" (EGSR 2013),
// as given in Pharr/Jakob/Humphreys, Physically Based Rendering (4th ed.) Sec 12.5.3 -- an exact,
// constant-pdf solid-angle sampler for a rectangular light, avoiding the variance a naive uniform-
// area sample-then-reweight strategy pays wherever the rectangle subtends very different solid
// angles across its own extent (a receiver near one edge, say). buildSphericalRectangle and
// sample()/LightSet::pdfQuad below all derive the SAME solid angle, so a light's pdf can never
// describe a different measure than the direction its own sampler actually drew -- the same f/pdf
// discipline environment_map.cpp's equirectTexelOf already enforces for the environment light.
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

// The set of lights NEE can sample from in one renderPathTraced() pass: the environment map (or none
// -- the HUD's environment-light toggle off) plus zero or more rectangular emitters. Selection is
// uniform over whichever of these are present -- one categorical draw when there are 2+ lights, no
// draw at all when there is exactly 1 (a degenerate one-element categorical needs no random
// variate), which is what keeps a single-environment/no-quad scene's sample sequence -- and
// therefore its rendered image -- bit-identical to a renderer with no light-selection mechanism at
// all. Built once per renderPathTraced() pass from that pass's env/quad state and held by const reference through the whole tile loop -- a non-owning view, not a value type: it holds references, so it is non-assignable and every referent must outlive it.
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

}  // namespace engine::scene
