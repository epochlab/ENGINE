# AOV reference

[← README](../README.md)


The full set the viewer's dropdown and `render_beauty --aov` name, defined in `include/engine/debug/aov.h` and grouped here by category. The 13 primary-hit-only AOVs are produced synchronously by the CPU rasterizer every frame (`rasterizer.h`); the other 14 need light-transport data and converge progressively with Beauty (`aovNeedsLightTransport`).

## Utility

| AOV | Mechanism |
|---|---|
| Beauty | Final accumulated radiance, post tone-mapping — the primary output |
| Wireframe | Screen-space line rasterization (Pineda 1988), z-tested against the scene's own depth: white mesh-triangle edges, plus one bounding box per instance in that instance's `falseColorForId` hue, the same hue ObjectID gives it (drawn on top, so the box wins) — visualizes triangle density/topology and each object's extent/placement in one view |
| Alpha | 1.0 on a primary hit, 0.0 on a primary miss — a real coverage mask (this renderer isn't opaque-only-by-construction) |
| Depth | Planar camera-space Z (Arnold/RenderMan/EXR "Z" convention) at the primary hit, for depth-based compositing/debugging |
| HSV | Colour-space transform of Beauty — isolates hue/saturation shifts a pure RGB view can hide |
| Luminance | Rec.709 luminance of Beauty — isolates perceived brightness from colour |
| Sobel | 3×3 Sobel gradient magnitude of Luminance — a cheap edge/gradient signal |
| Gabor | 4-orientation Gabor kernel bank, max response, of Luminance — directional edge/texture response Sobel's isotropic magnitude can't distinguish |
| WorldPos | Raw world-space primary-hit position, for debugging geometry/UV placement independent of shading |
| UV | Primary-hit interpolated UV (fractional part) — visualizes the texture-space mapping directly |

## Material

| AOV | Mechanism |
|---|---|
| Normal | Shading (normal-mapped) normal at the primary hit — the normal actually used in shading |
| GeomNormal | Smooth interpolated vertex normal, before normal-mapping — separates a bad normal map from a bad base mesh |
| Albedo | Base-colour texture sample at the primary hit — isolates texture data from lighting |
| Metallic | Per-instance metallic factor (`settings.metallicFactor`, `materials/*.json` or `SceneConfig::materialOverrides`), uniform within one object's triangles but no longer whole-image-constant now that per-object material assignment exists — debugs which instance carries which metallic value |
| Roughness | Roughness texture × a per-instance factor, floored at that material's own `roughnessMin` (materials can set their own floor, e.g. glass's below diffuse/chrome's shared 0.045); texture varies per hit, factor/floor vary per instance — debugs material authoring independent of shading |
| Tangent | Shading tangent basis at the primary hit — debugs the tangent-space basis used for normal mapping |
| ObjectID | Per-instance index, false-coloured (`falseColorForId`) — an isolation mask for compositing/debugging |
| AO | Cosine-weighted obscurance (Zhukov et al. 1998; Iones et al. 2003), the distance-weighted generalisation of ambient occlusion (Miller 1994; Landis 2002). One hemisphere ray per sample bounded by `aoMaxDistance` (`profile.json`), each hit weighted `1 - (1 - t/aoMaxDistance)^2` so occlusion grades with proximity and reaches full visibility smoothly at the bound. 1.0 = unoccluded, the opposite polarity to Shadow — reads contact/corner darkening off the actual geometry, independent of material and lighting |

## Transport

| AOV | Mechanism |
|---|---|
| Fresnel | Expected Fresnel reflectance over the **visible microfacet normal distribution**, `E[F(wo.wh)]` for `wh ~ D_vis(wo)`. One VNDF draw per sample (Heitz 2018, the same `D_vis` and `alpha` `sampleBsdf` draws from), through `mix(exact dielectric Fresnel, exact complex-IOR conductor Fresnel, metallic)` (`fresnelAtMicrofacet`, `bsdf.h`), progressive like every other path-traced lane. This is the angle the microfacet BSDF actually evaluates Fresnel at (Walter et al. 2007), so it is roughness-dependent where a macro-normal value cannot be. At `n.wo = 0.05` on an `ior` 1.5 dielectric it reads 0.7521 at the roughness floor, 0.4406 at roughness 0.3 and 0.1692 at 0.6, against a macro-normal 0.7521 throughout. Full RGB — a conductor's Fresnel is chromatic by construction (`edgeTint` inverts to a per-channel complex IOR), which the rasterizer's `(F, 1-F, 0)` packing discarded. Collapses onto the macro-normal value to 4 decimals as `alpha` reaches its `kMinAlpha` floor, so it is a strict generalisation of the AOV it replaces, not a different quantity |
| IOR | Per-instance dielectric IOR (`settings.ior`), -1 on a miss — isolates the raw refractive-index input driving Fresnel/transmission |
| BounceCount | Mean path termination depth across samples, per pixel — debugs Russian roulette/termination behaviour |

## Lighting

| AOV | Mechanism |
|---|---|
| DirectDiffuse | Diffuse-bucketed radiance from a path's first (bounce-0) surface, physical (base colour included) — isolates direct diffuse light arrival, in the same units as Beauty |
| IndirectDiffuse | Diffuse-bucketed radiance from later bounces — isolates indirect (bounced) diffuse contribution |
| DirectSpecular | Specular-reflection-bucketed radiance, one bounce from camera — isolates direct specular contribution |
| IndirectSpecular | Specular-reflection-bucketed radiance, later bounces — isolates indirect specular (reflections) |
| Refraction | Radiance from any path that sampled a transmission lobe (sticky bucket) — isolates glass/transmissive transport |
| Shadow | Binary NEE occlusion test toward the sampled light (environment or an area light, per `LightSet`'s uniform selection) at the primary hit, re-averaged across progressive passes into continuous shadow/penumbra density — isolates direct-light visibility from material/lighting colour |
