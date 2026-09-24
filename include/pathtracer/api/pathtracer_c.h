#ifndef PATHTRACER_API_PATHTRACER_C_H
#define PATHTRACER_API_PATHTRACER_C_H

/* Flat C ABI over api/headless_renderer.h for runtimes without C++; one PtRenderer per thread. See DERIVATIONS.md "Flat C ABI". */

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque scene handle: one loaded scene, its BVH, its thread pool and its reusable framebuffers. */
typedef struct PtRenderer PtRenderer;

#define PT_OK 0
#define PT_ERROR 1

/* Loads a scene; NULL on failure with a NUL-terminated reason in err (truncated to err_cap). scene_path is relative to asset_root. */
PtRenderer* pt_renderer_open(const char* asset_root, const char* scene_path, char* err, int err_cap);
void pt_renderer_close(PtRenderer* renderer);

/* The AOV table, exposed so a caller never hardcodes a parallel copy of it. Ids are dense in [0, pt_aov_count). */
int pt_aov_count(void);
const char* pt_aov_name(int aov);
/* Case- and separator-insensitive: "bounce-count", "bounce_count" and "bouncecount" all resolve. -1 if unknown. */
int pt_aov_id(const char* name);
/* Channels the AOV carries, what pt_render writes per texel: 1 for depth or a filter, 2 for UV, 3 for radiance. Not HdrImage's 4. */
int pt_aov_channels(int aov);
/* Non-zero if this AOV needs light transport, so a caller can tell which requests the `samples` field affects. */
int pt_aov_needs_samples(int aov);

/* Pose, lens and exposure. aperture/shutter_seconds/iso set exposure value ONLY: the camera is a pinhole, with no depth of field. */
typedef struct {
    float position[3];
    float yaw_degrees;
    float pitch_degrees;
    float film_back_mm[2]; /* sensor gate width, height */
    float focal_length_mm;
    float near_clip;
    float far_clip;
    float aperture;
    float shutter_seconds;
    float iso;
} PtCamera;

/* profile.json's authored camera and window size -- the defaults a caller overrides one field at a time. */
void pt_renderer_default_camera(const PtRenderer* renderer, PtCamera* out);
int pt_renderer_default_width(const PtRenderer* renderer);
int pt_renderer_default_height(const PtRenderer* renderer);

typedef struct {
    PtCamera camera;
    int width;
    int height;
    /* Path-traced passes at one sample each, averaged. Ignored by a request whose AOVs are all rasterizer-backed. */
    int samples;
    /* Fixes the sampler's scramble. The same seed and request reproduce the same floats exactly. */
    unsigned int seed;
    const int* aovs;
    int aov_count;
} PtRenderRequest;

/* Renders every requested AOV. out parallels request->aovs at width * height * pt_aov_channels(aovs[i]) floats, row-major top-left. */
int pt_render(PtRenderer* renderer, const PtRenderRequest* request, float* const* out, char* err, int err_cap);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif  /* PATHTRACER_API_PATHTRACER_C_H */
