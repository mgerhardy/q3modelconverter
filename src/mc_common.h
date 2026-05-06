/*
===========================================================================
modelconverter - shared definitions
===========================================================================
*/

#ifndef MC_COMMON_H
#define MC_COMMON_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

/* MSVC compat: POSIX names are not available. */
#ifdef _MSC_VER
#include <malloc.h>
#define strcasecmp  _stricmp
#define strncasecmp _strnicmp
#define strdup      _strdup
#define alloca      _alloca
#include <sys/stat.h>
#ifndef S_ISDIR
#define S_ISDIR(m) (((m) & _S_IFMT) == _S_IFDIR)
#endif
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MC_MAX_PATH 1024
#define MC_MAX_QPATH 64

/* MD3 constants. */
#define MD3_IDENT (('3' << 24) + ('P' << 16) + ('D' << 8) + 'I')
#define MD3_VERSION 15
#define MD3_MAX_LODS 3
#define MD3_MAX_TRIANGLES 8192
#define MD3_MAX_VERTS 4096
#define MD3_MAX_SHADERS 256
#define MD3_MAX_FRAMES 1024
#define MD3_MAX_SURFACES 32
#define MD3_MAX_TAGS 16
#define MD3_XYZ_SCALE (1.0f / 64.0f)
#define MD3_SURFACE_IDENT MD3_IDENT

/* IQM constants. */
#define IQM_MAGIC "INTERQUAKEMODEL"
#define IQM_VERSION 2

/* MDR constants. */
#define MDR_IDENT (('5' << 24) + ('M' << 16) + ('D' << 8) + 'R')
#define MDR_VERSION 2
#define MDR_MAX_BONES 128

typedef float vec2_t[2];
typedef float vec3_t[3];
typedef float vec4_t[4];

#pragma pack(push, 1)
typedef struct {
	float bounds[2][3];
	float localOrigin[3];
	float radius;
	char name[16];
} mc_md3Frame_t;

typedef struct {
	char name[MC_MAX_QPATH];
	float origin[3];
	float axis[3][3];
} mc_md3Tag_t;

typedef struct {
	int ident;
	char name[MC_MAX_QPATH];
	int flags;
	int numFrames;
	int numShaders;
	int numVerts;
	int numTriangles;
	int ofsTriangles;
	int ofsShaders;
	int ofsSt;
	int ofsXyzNormals;
	int ofsEnd;
} mc_md3Surface_t;

typedef struct {
	char name[MC_MAX_QPATH];
	int shaderIndex;
} mc_md3Shader_t;

typedef struct {
	int indexes[3];
} mc_md3Triangle_t;

typedef struct {
	float st[2];
} mc_md3St_t;

typedef struct {
	short xyz[3];
	short normal;
} mc_md3XyzNormal_t;

typedef struct {
	int ident;
	int version;
	char name[MC_MAX_QPATH];
	int flags;
	int numFrames;
	int numTags;
	int numSurfaces;
	int numSkins;
	int ofsFrames;
	int ofsTags;
	int ofsSurfaces;
	int ofsEnd;
} mc_md3Header_t;

/* MDR mirrors. Variable sized arrays are written/read manually. */
typedef struct {
	int boneIndex;
	float boneWeight;
	float offset[3];
} mc_mdrWeight_t;

typedef struct {
	float normal[3];
	float texCoords[2];
	int numWeights;
	/* mc_mdrWeight_t weights[numWeights] follows */
} mc_mdrVertexHeader_t;

typedef struct {
	int indexes[3];
} mc_mdrTriangle_t;

typedef struct {
	int ident;
	char name[MC_MAX_QPATH];
	char shader[MC_MAX_QPATH];
	int shaderIndex;
	int ofsHeader;
	int numVerts;
	int ofsVerts;
	int numTriangles;
	int ofsTriangles;
	int numBoneReferences;
	int ofsBoneReferences;
	int ofsEnd;
} mc_mdrSurface_t;

typedef struct {
	float matrix[3][4];
} mc_mdrBone_t;

typedef struct {
	float bounds[2][3];
	float localOrigin[3];
	float radius;
	char name[16];
	/* mc_mdrBone_t bones[numBones] follows */
} mc_mdrFrameHeader_t;

typedef struct {
	int numSurfaces;
	int ofsSurfaces;
	int ofsEnd;
} mc_mdrLOD_t;

typedef struct {
	int boneIndex;
	char name[32];
} mc_mdrTag_t;

typedef struct {
	int ident;
	int version;
	char name[MC_MAX_QPATH];
	int numFrames;
	int numBones;
	int ofsFrames;
	int numLODs;
	int ofsLODs;
	int numTags;
	int ofsTags;
	int ofsEnd;
} mc_mdrHeader_t;
#pragma pack(pop)

/* In-memory generic model representation used for conversions. */
/* Quake3 player part identifier ("head", "upper", "lower" or "" for a
   plain non-bundled model).  Stamped on every surface and tag when a
   player directory is loaded so the splitter can rebuild three MD3s. */
#define MC_PART_NONE  ""
#define MC_PART_HEAD  "head"
#define MC_PART_UPPER "upper"
#define MC_PART_LOWER "lower"

typedef struct {
	char name[MC_MAX_QPATH];
	char part[16];			   /* "" / "head" / "upper" / "lower" */
	int  lod;                  /* 0 = base detail, 1..MD3_MAX_LODS-1 = lower detail */
	char shader[MC_MAX_QPATH]; /* shader path (texture path without extension) */
	char texture[MC_MAX_PATH]; /* on-disk texture file referenced by the material */

	/* Material properties carried through round-trips and used to author a
	   companion .shader file when --shader is requested. */
	int two_sided;					/* 0/1 - emits "cull disable" */
	int alpha_mode;					/* 0=opaque, 1=mask, 2=blend */
	float alpha_cutoff;				/* used when alpha_mode == mask */
	float base_color[4];			/* baseColorFactor (RGBA, 0..1) */
	char normal_map[MC_MAX_PATH];		   /* glTF normalTexture or "_n" companion */
	char normal_height_map[MC_MAX_PATH];   /* "_nh" companion (IMGTYPE_NORMALHEIGHT) */
	char emissive_map[MC_MAX_PATH]; /* glTF emissiveTexture source */
	char mr_map[MC_MAX_PATH];		/* glTF metallicRoughnessTexture source */
	char occlusion_map[MC_MAX_PATH]; /* glTF occlusionTexture source */
	float emissive_factor[3];
	float metallic_factor;			/* glTF metallicFactor (default 1.0) */
	float roughness_factor;			/* glTF roughnessFactor (default 1.0) */
	float occlusion_strength;		/* glTF occlusionTexture.strength (default 1.0) */
	char surfaceparm[64]; /* extra surfaceparm token (e.g. "trans", "nonsolid") */
	char deform[32];	  /* shader deform that constrains topology
							 (e.g. "autosprite", "autosprite2") -
							 surfaces with a deform set must NOT be
							 subdivided/decimated, the engine relies
							 on the original quad layout. */
	/* Raw Q3 shader body (everything between the outer { }, NUL-terminated)
	   captured when a .shader stanza was matched on read. Allows us to
	   round-trip the full set of stages / blendFuncs / tcMods / rgbGens
	   verbatim through glTF (carried in extras as "q3_body_b64") and
	   re-emit them when writing a .shader file. NULL if no source body
	   was available. Owned by the surface, freed by mc_model_free. */
	char *q3_shader_body;

	int numVerts;
	int numTris;

	/* Per-frame interleaved arrays:
	   xyz:    numFrames * numVerts * 3 floats (object space)
	   normal: numFrames * numVerts * 3 floats (object space, unit length) */
	float *xyz;
	float *normal;
	float *st;	  /* numVerts * 2 floats (shared across frames, classic MD3) */
	int *indices; /* numTris * 3 ints */

	/* Optional skeletal-skinning data preserved for IQM/MDR/glTF
	   round-trips.  Allocated only when the source format actually
	   provides per-vertex bone weights (the existing per-frame xyz/normal
	   arrays remain authoritative for rendering / vertex-anim formats).
	     blendIndices: numVerts * 4 unsigned bytes (joint indices into mc_model_t::joints)
	     blendWeights: numVerts * 4 floats summing to ~1 per vertex
	   blendWeights[k] == 0 means slot k is unused. */
	unsigned char *blendIndices;
	float *blendWeights;
} mc_surface_t;

typedef struct {
	char name[MC_MAX_QPATH];
	char part[16]; /* part owning this tag (which MD3 wrote it) */
	int  lod;      /* LOD index of the part this tag was emitted from */
	float origin[3];
	float axis[3][3]; /* row-major; axis[0]=forward, axis[1]=left, axis[2]=up */
} mc_tag_t;

/* A single line from a Quake3 .skin file: "<surface>,<shader>". */
typedef struct {
	char surface[MC_MAX_QPATH];
	char shader[MC_MAX_QPATH];
} mc_skin_entry_t;

/* A named .skin variant for one player part.  `name` is the variant
   ("default", "red", "blue", "padknight", ...), `part` is "head" /
   "upper" / "lower" (or "" for a non-player model). */
typedef struct {
	char name[MC_MAX_QPATH];
	char part[16];
	mc_skin_entry_t *entries;
	int numEntries;
} mc_skin_variant_t;

typedef struct {
	char name[MC_MAX_QPATH];
	float bounds[2][3];
	float localOrigin[3];
	float radius;
} mc_frame_t;

/* Quake3 player animation entry (one row from animation.cfg). */
typedef struct {
	char name[64];	  /* canonical name, e.g. "TORSO_ATTACK" */
	int firstFrame;	  /* absolute frame index (matches animation.cfg) */
	int numFrames;	  /* number of animation frames */
	int loopFrames;	  /* >0 means looping starts at firstFrame+(numFrames-loopFrames) */
	float fps;		  /* sample rate */
} mc_animation_t;

/* Optional skeleton.  Joint poses are stored TRS so we can round-trip an
   IQM frame channel stream losslessly and convert directly to/from a
   glTF skin's animated joint nodes. */
typedef struct {
	char  name[64];
	int   parent;          /* -1 for root */
	float bindTrans[3];
	float bindRot[4];      /* xyzw quaternion */
	float bindScale[3];
} mc_joint_t;

typedef struct {
	float trans[3];
	float rot[4];          /* xyzw quaternion */
	float scale[3];
} mc_joint_pose_t;

typedef struct {
	int numFrames;
	int numSurfaces;
	int numTags;
	float fps;

	mc_frame_t *frames;		/* numFrames */
	mc_surface_t *surfaces; /* numSurfaces; each surface has numFrames worth of xyz/normal */
	mc_tag_t *tags;			/* numFrames * numTags */

	/* Quake3 player animation.cfg metadata (optional). */
	mc_animation_t *animations;
	int numAnimations;
	char anim_sex;					/* 'm' / 'f' / 'n', 0 if unset */
	float anim_headoffset[3];		/* default 0,0,0 */
	int anim_has_headoffset;		/* 1 if the file had a `headoffset` line */

	/* Set when this model represents a full Q3 player bundle
	   (head + upper + lower).  Each surface and tag carries a `part`
	   label and the splitter will produce three MD3s on save. */
	int is_player_bundle;

	/* Optional per-part frame counts so the splitter knows how many
	   frames each MD3 should hold.  Indexed by part:
		 0=head, 1=upper, 2=lower. */
	int part_numFrames[3];

	/* All discovered .skin variants (default + team + visual variants).
	   The splitter writes them back out with the produced MD3s; the
	   loader collects them so they survive a round-trip. */
	mc_skin_variant_t *skins;
	int numSkins;

	/* Optional skeleton.  When numJoints > 0 surfaces may carry per-vertex
	   blendIndices / blendWeights and jointPoses holds one TRS pose per
	   (frame, joint).  Vertex-anim formats (MD3 / glTF morph) ignore
	   these; IQM / MDR / glTF-skin emit them losslessly when present. */
	mc_joint_t      *joints;       /* numJoints */
	int              numJoints;
	mc_joint_pose_t *jointPoses;   /* numFrames * numJoints */

	/* Number of LOD levels detected on input.  LOD0 is the highest
	   detail; surfaces tag their level via mc_surface_t::lod.  Defaults
	   to 0; writers that support multi-LOD output (MD3, MDR) compute
	   their own LOD count from the surface tags but use this hint when
	   emitting --info. */
	int numLODs;
} mc_model_t;

/* IO helpers. */
unsigned char *mc_read_file(const char *path, size_t *out_size);
int mc_write_file(const char *path, const void *data, size_t size);
void mc_strip_extension(char *path);
const char *mc_basename(const char *path);
void mc_dirname(const char *path, char *out, size_t out_size);
void mc_ensure_directory(const char *path);
void mc_join_path(char *out, size_t out_size, const char *a, const char *b);
void mc_q_strncpy(char *dst, const char *src, size_t n);

/* Base64 encoder. Returns a malloc'd, NUL-terminated string with the standard
   RFC 4648 alphabet and '=' padding. Caller frees. NULL on allocation failure. */
char *mc_b64_encode(const void *data, size_t len);

/* Base64 decoder (RFC 4648). Skips whitespace, accepts trailing '='. Returns
   malloc'd buffer (caller frees) and writes its size to out_len. NULL on
   allocation failure. Output is NOT NUL-terminated. */
unsigned char *mc_b64_decode(const char *in, size_t in_len, size_t *out_len);

/* Quiet variant of mc_read_file: returns NULL on missing file without
   logging anything. Use this for speculative probes (companion textures,
   shader-path scans) where misses are expected and noise. */
unsigned char *mc_read_file_quiet(const char *path, size_t *out_size);

/* Math helpers. */
short mc_normal_to_latlong(const float n[3]);
void mc_latlong_to_normal(short packed, float out[3]);
void mc_compute_bounds(const float *xyz, int numVerts, float mins[3], float maxs[3], float *outRadius);

/* Model lifecycle. */
void mc_model_init(mc_model_t *m);
void mc_model_free(mc_model_t *m);
mc_surface_t *mc_model_add_surface(mc_model_t *m);
mc_frame_t *mc_model_add_frame(mc_model_t *m);
mc_tag_t *mc_model_add_tag_slot(mc_model_t *m, int numFrames);
void mc_surface_alloc(mc_surface_t *s, int numVerts, int numTris, int numFrames);

/* Allocate the optional per-vertex blendIndices / blendWeights arrays
   on a surface.  Slots default to 0 indices / 0 weights. */
void mc_surface_alloc_blend(mc_surface_t *s);

/* Allocate (or resize) the model's joint array and the per-frame
   jointPoses array.  Existing entries are preserved when growing. */
void mc_model_set_joints(mc_model_t *m, int numJoints);

/* Format detection. */
typedef enum { MC_FMT_UNKNOWN, MC_FMT_GLTF, MC_FMT_GLB, MC_FMT_MD3, MC_FMT_IQM, MC_FMT_MDR, MC_FMT_ASE, MC_FMT_3DS } mc_format_t;

mc_format_t mc_guess_format(const char *path);
const char *mc_format_name(mc_format_t f);

/* Conversion entry points (each returns 0 on success). */
int mc_load_gltf(const char *path, mc_model_t *out, float fps_hint);
/* `asset_roots` is an optional NULL-terminated array of directories
   searched for textures referenced by surfaces.  When the file is found
   it is either embedded into the .glb binary chunk or copied next to
   the .gltf output. */
int mc_save_gltf(const char *path, const mc_model_t *m, int as_glb, const char *const *asset_roots);

int mc_load_md3(const char *path, mc_model_t *out);
int mc_save_md3(const char *path, const mc_model_t *m);

int mc_load_iqm(const char *path, mc_model_t *out);
int mc_save_iqm(const char *path, const mc_model_t *m);

int mc_load_mdr(const char *path, mc_model_t *out);
int mc_save_mdr(const char *path, const mc_model_t *m);

int mc_load_ase(const char *path, mc_model_t *out);
int mc_save_ase(const char *path, const mc_model_t *m);

int mc_load_3ds(const char *path, mc_model_t *out);
int mc_save_3ds(const char *path, const mc_model_t *m);

/* Skin file output (Quake3 .skin format). */
int mc_save_skin(const char *path, const mc_model_t *m);

/* Q3 shader file (.shader) helpers. */
int mc_save_q3shader(const char *path, const mc_model_t *m);
int mc_load_q3shader(const char *path, mc_model_t *m);

/* Player bundle support (head.md3 + upper.md3 + lower.md3 +
   animation.cfg + .skin / .shader sidecars).  When loading, the
   resulting mc_model_t has is_player_bundle == 1; surfaces and tags
   carry a `part` label and skin variants are collected into
   `m->skins`.  When saving, three MD3s + animation.cfg + every
   collected .skin variant are written under `dir`.

   `mc_is_player_dir` returns 1 when `dir` looks like a player
   directory (contains lower.md3 or any *_player layout marker). */
int mc_is_player_dir(const char *dir);
int mc_load_player_dir(const char *dir, mc_model_t *out);
int mc_save_player_dir(const char *dir, const mc_model_t *m);

/* Skin variant management. */
mc_skin_variant_t *mc_model_add_skin_variant(mc_model_t *m, const char *name, const char *part);
int mc_load_q3skin_variant(const char *path, mc_skin_variant_t *out);
int mc_save_q3skin_variant(const char *path, const mc_skin_variant_t *v);

/* Q3 skin file (.skin) loader: parses "surface,texture" lines and
   updates each matching surface's shader+texture so subsequent shader
   resolution and texture lookup pick up the real material name. */
int mc_load_q3skin(const char *path, mc_model_t *m);

/* Loop subdivision (the triangle-mesh equivalent of Catmull-Clark).
   Refines every surface's mesh and every animation frame `iterations`
   times.  UVs are linearly interpolated, normals are recomputed per
   frame, per-frame bounds are refreshed.  Topology is rebuilt from
   frame 0 and applied uniformly to all frames so morph targets stay
   coherent.  Returns 0 on success. */
int mc_subdivide(mc_model_t *m, int iterations);

/* Garland-Heckbert quadric error metric edge-collapse decimation,
   matching Blender's "Decimate > Collapse" modifier.  `ratio` is the
   fraction of triangles to keep per surface (0..1].  Topology is
   resolved on frame 0; the same collapse sequence and edge parameters
   are applied to every frame so animated meshes stay coherent.  UVs
   are interpolated, normals are recomputed.  Returns 0 on success. */
int mc_decimate(mc_model_t *m, float ratio);

/* For every surface that currently has lod==0, generate `numLods`
   additional lower-detail copies (lod = 1..numLods) by duplicating the
   surface and decimating it with the matching ratio in `ratios` (e.g.
   {0.5f, 0.25f}).  Useful to auto-author MD3 LOD chains
   (<part>.md3 / <part>_1.md3 / <part>_2.md3) for assets that ship
   only a base detail level.  Tags are not duplicated (the engine only
   reads tags from the base LOD).  numLods is clamped to MD3_MAX_LODS-1. */
int mc_gen_lods(mc_model_t *m, int numLods, const float *ratios);

/* Walk the given asset roots looking for .shader files and apply each
   to the model.  Surfaces whose shader name matches a stanza inside
   any discovered .shader file inherit cull/blend/normalmap/etc. */
int mc_autoload_shaders(const char *const *asset_roots, mc_model_t *m);
void mc_q3shader_set_max_depth(int depth);
void mc_q3shader_add_search_path(const char *dir);

/* Decode an image (TGA/BMP/JPEG/PNG/...) from memory and re-encode it
   as PNG.  Caller frees the returned buffer.  Returns NULL on failure. */
unsigned char *mc_image_to_png(const unsigned char *src, size_t src_size, size_t *out_size);
unsigned char *mc_image_make_missing_placeholder(size_t *out_size);

/* Quake3 player animation.cfg I/O. */
int mc_load_animation_cfg(const char *path, mc_model_t *m);
int mc_save_animation_cfg(const char *path, const mc_model_t *m);
mc_animation_t *mc_model_add_animation(mc_model_t *m);

/* PK3 (zip) archive reading. */
int mc_pk3_is_pk3(const char *path);
unsigned char *mc_pk3_read_entry(const char *pk3_path, const char *entry_name, size_t *out_size);
int mc_pk3_for_each(const char *pk3_path, const char *suffix,
                    void (*cb)(const char *pk3, const char *entry, void *user), void *user);

/* Verbosity flag.
   - mc_verbose <  0  : quiet  (errors only)
   - mc_verbose == 0  : default (errors + warnings + info)
   - mc_verbose >  0  : verbose (errors + warnings + info + debug) */
extern int mc_verbose;

#define MC_DEBUG(...)                                                                                                  \
	do {                                                                                                               \
		if (mc_verbose > 0) {                                                                                          \
			fprintf(stderr, "[debug] ");                                                                               \
			fprintf(stderr, __VA_ARGS__);                                                                              \
		}                                                                                                              \
	} while (0)
#define MC_INFO(...)                                                                                                   \
	do {                                                                                                               \
		if (mc_verbose >= 0) {                                                                                         \
			fprintf(stderr, __VA_ARGS__);                                                                              \
		}                                                                                                              \
	} while (0)
#define MC_WARN(...)                                                                                                   \
	do {                                                                                                               \
		if (mc_verbose >= 0) {                                                                                         \
			fprintf(stderr, "[warn] ");                                                                                \
			fprintf(stderr, __VA_ARGS__);                                                                              \
		}                                                                                                              \
	} while (0)
#define MC_ERR(...)                                                                                                    \
	do {                                                                                                               \
		fprintf(stderr, "[error] ");                                                                                   \
		fprintf(stderr, __VA_ARGS__);                                                                                  \
	} while (0)
/* Backwards-compat: existing call sites stay verbose-gated like before. */
#define MC_LOG MC_DEBUG

/* Bounds-check helper for binary file readers.  Verifies that the byte
   range [ofs, ofs+len) falls entirely within [0, filesize).  Evaluates
   to 1 on success, 0 on failure.  Handles unsigned overflow safely. */
static inline int mc_bounds_check(size_t ofs, size_t len, size_t filesize) {
	return (ofs <= filesize && len <= filesize - ofs);
}

/* Checked multiplication for allocation sizes.  Returns a*b if it fits
   in size_t, or 0 on overflow (which mc_calloc/mc_malloc will then
   handle as a zero-byte allocation — safe but useless, so callers
   should validate counts before reaching this point). */
static inline size_t mc_safe_mul(size_t a, size_t b) {
	if (a != 0 && b > (size_t)-1 / a) return 0;
	return a * b;
}
static inline size_t mc_safe_mul3(size_t a, size_t b, size_t c) {
	return mc_safe_mul(mc_safe_mul(a, b), c);
}

/* Safe allocation wrappers.  Abort with a diagnostic on OOM so callers
   never have to NULL-check.  Use these instead of raw malloc/calloc/realloc
   in all project .c files (vendored headers like cgltf.h are excluded). */
static inline void *mc_malloc(size_t size) {
	void *p = malloc(size);
	if (!p && size) { fprintf(stderr, "[fatal] out of memory (malloc %zu)\n", size); abort(); }
	return p;
}
static inline void *mc_calloc(size_t count, size_t size) {
	void *p = calloc(count, size);
	if (!p && count && size) { fprintf(stderr, "[fatal] out of memory (calloc %zu*%zu)\n", count, size); abort(); }
	return p;
}
static inline void *mc_realloc(void *ptr, size_t size) {
	void *p = realloc(ptr, size);
	if (!p && size) { fprintf(stderr, "[fatal] out of memory (realloc %zu)\n", size); abort(); }
	return p;
}

#endif /* MC_COMMON_H */
