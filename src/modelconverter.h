/*
===========================================================================
modelconverter - public API

This header exposes only the types and functions needed by external
consumers linking against libmodelconverter.  Internal helpers, logging
macros and format-specific on-disk structs live in mc_common.h.
===========================================================================
*/

#ifndef MODELCONVERTER_H
#define MODELCONVERTER_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Core types ---- */

#define MC_MAX_PATH  1024
#define MC_MAX_QPATH 64

typedef struct mc_surface_t  mc_surface_t;
typedef struct mc_tag_t      mc_tag_t;
typedef struct mc_frame_t    mc_frame_t;
typedef struct mc_model_t    mc_model_t;

/* ---- Model lifecycle ---- */

void mc_model_init(mc_model_t *m);
void mc_model_free(mc_model_t *m);

/* ---- Format detection ---- */

typedef enum { MC_FMT_UNKNOWN, MC_FMT_GLTF, MC_FMT_GLB, MC_FMT_MD3, MC_FMT_IQM, MC_FMT_MDR, MC_FMT_3DS } mc_format_t;

mc_format_t mc_guess_format(const char *path);
const char *mc_format_name(mc_format_t f);

/* ---- Load / save ---- */

int mc_load_gltf(const char *path, mc_model_t *out, float fps_hint);
int mc_save_gltf(const char *path, const mc_model_t *m, int as_glb, const char *const *asset_roots);

int mc_load_md3(const char *path, mc_model_t *out);
int mc_save_md3(const char *path, const mc_model_t *m);

int mc_load_iqm(const char *path, mc_model_t *out);
int mc_save_iqm(const char *path, const mc_model_t *m);

int mc_load_mdr(const char *path, mc_model_t *out);
int mc_save_mdr(const char *path, const mc_model_t *m);

int mc_load_3ds(const char *path, mc_model_t *out);
int mc_save_3ds(const char *path, const mc_model_t *m);

/* ---- Sidecars ---- */

int mc_save_skin(const char *path, const mc_model_t *m);
int mc_save_q3shader(const char *path, const mc_model_t *m);
int mc_load_q3shader(const char *path, mc_model_t *m);
int mc_load_q3skin(const char *path, mc_model_t *m);

/* ---- Player bundles ---- */

int mc_is_player_dir(const char *dir);
int mc_load_player_dir(const char *dir, mc_model_t *out);
int mc_save_player_dir(const char *dir, const mc_model_t *m);

/* ---- Mesh processing ---- */

int mc_subdivide(mc_model_t *m, int iterations);
int mc_decimate(mc_model_t *m, float ratio);
int mc_gen_lods(mc_model_t *m, int numLods, const float *ratios);

/* ---- Verbosity (default 0 = info; <0 = quiet; >0 = verbose) ---- */

extern int mc_verbose;

#ifdef __cplusplus
}
#endif

#endif /* MODELCONVERTER_H */
