/*
===========================================================================
modelconverter - common helpers
===========================================================================
*/

#include "mc_common.h"

#include <ctype.h>
#include <sys/stat.h>
#ifdef _WIN32
#include <direct.h>
#define MC_MKDIR(p) _mkdir(p)
#define MC_PATHSEP '\\'
#else
#include <unistd.h>
#define MC_MKDIR(p) mkdir((p), 0755)
#define MC_PATHSEP '/'
#endif

int mc_verbose = 0;

/* Base64 (RFC 4648) - small, malloc-based; OK for the few-KB payloads
   we round-trip through glTF extras. */
static const char k_b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

char *mc_b64_encode(const void *data, size_t len) {
	const unsigned char *in = (const unsigned char *)data;
	size_t out_len = ((len + 2) / 3) * 4;
	char *out = (char *)mc_malloc(out_len + 1);
	if (!out)
		return NULL;
	size_t i = 0, o = 0;
	while (i + 2 < len) {
		unsigned v = ((unsigned)in[i] << 16) | ((unsigned)in[i + 1] << 8) | (unsigned)in[i + 2];
		out[o++] = k_b64[(v >> 18) & 0x3f];
		out[o++] = k_b64[(v >> 12) & 0x3f];
		out[o++] = k_b64[(v >> 6) & 0x3f];
		out[o++] = k_b64[v & 0x3f];
		i += 3;
	}
	if (i < len) {
		unsigned v = (unsigned)in[i] << 16;
		if (i + 1 < len)
			v |= (unsigned)in[i + 1] << 8;
		out[o++] = k_b64[(v >> 18) & 0x3f];
		out[o++] = k_b64[(v >> 12) & 0x3f];
		out[o++] = (i + 1 < len) ? k_b64[(v >> 6) & 0x3f] : '=';
		out[o++] = '=';
	}
	out[o] = 0;
	return out;
}

static int mc__b64_value(int c) {
	if (c >= 'A' && c <= 'Z')
		return c - 'A';
	if (c >= 'a' && c <= 'z')
		return c - 'a' + 26;
	if (c >= '0' && c <= '9')
		return c - '0' + 52;
	if (c == '+')
		return 62;
	if (c == '/')
		return 63;
	return -1;
}

unsigned char *mc_b64_decode(const char *in, size_t in_len, size_t *out_len) {
	while (in_len > 0 && (in[in_len - 1] == '=' || isspace((unsigned char)in[in_len - 1])))
		in_len--;
	size_t cap = in_len * 3 / 4 + 4;
	unsigned char *out = (unsigned char *)mc_malloc(cap);
	if (!out)
		return NULL;
	size_t o = 0;
	int bits = 0, val = 0;
	for (size_t i = 0; i < in_len; ++i) {
		int c = (unsigned char)in[i];
		if (isspace(c))
			continue;
		int v = mc__b64_value(c);
		if (v < 0)
			continue;
		val = (val << 6) | v;
		bits += 6;
		if (bits >= 8) {
			bits -= 8;
			out[o++] = (unsigned char)((val >> bits) & 0xff);
		}
	}
	if (out_len)
		*out_len = o;
	return out;
}

unsigned char *mc_read_file(const char *path, size_t *out_size) {
	FILE *f = fopen(path, "rb");
	if (!f) {
		MC_ERR("cannot open '%s' for reading\n", path);
		return NULL;
	}
	if (fseek(f, 0, SEEK_END) != 0) {
		fclose(f);
		return NULL;
	}
	long sz = ftell(f);
	if (sz < 0) {
		fclose(f);
		return NULL;
	}
	rewind(f);
	unsigned char *buf = (unsigned char *)mc_malloc((size_t)sz + 1);
	if (!buf) {
		fclose(f);
		return NULL;
	}
	size_t n = fread(buf, 1, (size_t)sz, f);
	fclose(f);
	if (n != (size_t)sz) {
		free(buf);
		return NULL;
	}
	buf[sz] = 0;
	if (out_size) {
		*out_size = (size_t)sz;
	}
	return buf;
}

unsigned char *mc_read_file_quiet(const char *path, size_t *out_size) {
	FILE *f = fopen(path, "rb");
	if (!f)
		return NULL;
	if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return NULL; }
	long sz = ftell(f);
	if (sz < 0) { fclose(f); return NULL; }
	rewind(f);
	unsigned char *buf = (unsigned char *)mc_malloc((size_t)sz + 1);
	if (!buf) { fclose(f); return NULL; }
	size_t n = fread(buf, 1, (size_t)sz, f);
	fclose(f);
	if (n != (size_t)sz) { free(buf); return NULL; }
	buf[sz] = 0;
	if (out_size) *out_size = (size_t)sz;
	return buf;
}

int mc_write_file(const char *path, const void *data, size_t size) {
	FILE *f = fopen(path, "wb");
	if (!f) {
		MC_ERR("cannot open '%s' for writing\n", path);
		return -1;
	}
	size_t n = fwrite(data, 1, size, f);
	fclose(f);
	if (n != size) {
		MC_ERR("short write to '%s'\n", path);
		return -1;
	}
	return 0;
}

void mc_strip_extension(char *path) {
	char *dot = strrchr(path, '.');
	char *sep1 = strrchr(path, '/');
	char *sep2 = strrchr(path, '\\');
	char *sep = sep1 > sep2 ? sep1 : sep2;
	if (dot && (!sep || dot > sep)) {
		*dot = 0;
	}
}

const char *mc_basename(const char *path) {
	const char *s1 = strrchr(path, '/');
	const char *s2 = strrchr(path, '\\');
	const char *s = s1 > s2 ? s1 : s2;
	return s ? s + 1 : path;
}

void mc_dirname(const char *path, char *out, size_t out_size) {
	const char *s1 = strrchr(path, '/');
	const char *s2 = strrchr(path, '\\');
	const char *s = s1 > s2 ? s1 : s2;
	if (!s) {
		mc_q_strncpy(out, ".", out_size);
		return;
	}
	size_t n = (size_t)(s - path);
	if (n >= out_size)
		n = out_size - 1;
	memcpy(out, path, n);
	out[n] = 0;
}

void mc_ensure_directory(const char *path) {
	if (!path || !*path)
		return;
	struct stat st;
	if (stat(path, &st) == 0)
		return;
	MC_MKDIR(path);
}

void mc_join_path(char *out, size_t out_size, const char *a, const char *b) {
	if (!a || !*a) {
		mc_q_strncpy(out, b ? b : "", out_size);
		return;
	}
	if (!b || !*b) {
		mc_q_strncpy(out, a, out_size);
		return;
	}
	size_t la = strlen(a);
	int needSep = (la > 0 && a[la - 1] != '/' && a[la - 1] != '\\');
	snprintf(out, out_size, "%s%s%s", a, needSep ? "/" : "", b);
}

void mc_q_strncpy(char *dst, const char *src, size_t n) {
	if (!n)
		return;
	size_t i = 0;
	for (; i + 1 < n && src[i]; ++i) {
		dst[i] = src[i];
	}
	dst[i] = 0;
}

/*
=========================
Latitude / longitude packed normal encoding (matches the engine's MD3 path
in code/renderer_vulkan/tr_surface.c).
  decode X = cos(lat) * sin(lng)
  decode Y = sin(lat) * sin(lng)
  decode Z = cos(lng)
where lat / lng are 8-bit values that index a 256-entry sin table covering
the full circle.  The packed short stores (lat << 8) | lng.
=========================
*/
short mc_normal_to_latlong(const float n[3]) {
	float x = n[0], y = n[1], z = n[2];
	float len = sqrtf(x * x + y * y + z * z);
	if (len < 1e-6f) {
		/* Up. */
		return 0;
	}
	x /= len;
	y /= len;
	z /= len;

	if (z > 0.999999f)
		return 0; /* lng = 0    -> +Z */
	if (z < -0.999999f)
		return (short)128; /* lng = 0.5  -> -Z */

	float lng_rad = acosf(z);	  /* [0, pi]  */
	float lat_rad = atan2f(y, x); /* [-pi, pi]*/

	int lat = (int)(lat_rad * 255.0f / (2.0f * (float)M_PI));
	int lng = (int)(lng_rad * 255.0f / (2.0f * (float)M_PI));
	lat &= 0xff;
	lng &= 0xff;
	return (short)((lat << 8) | lng);
}

void mc_latlong_to_normal(short packed, float out[3]) {
	int lat = (packed >> 8) & 0xff;
	int lng = packed & 0xff;
	float lat_rad = (float)lat * (2.0f * (float)M_PI) / 255.0f;
	float lng_rad = (float)lng * (2.0f * (float)M_PI) / 255.0f;
	out[0] = cosf(lat_rad) * sinf(lng_rad);
	out[1] = sinf(lat_rad) * sinf(lng_rad);
	out[2] = cosf(lng_rad);
}

void mc_compute_bounds(const float *xyz, int numVerts, float mins[3], float maxs[3], float *outRadius) {
	mins[0] = mins[1] = mins[2] = 1e30f;
	maxs[0] = maxs[1] = maxs[2] = -1e30f;
	for (int i = 0; i < numVerts; ++i) {
		const float *p = xyz + i * 3;
		for (int k = 0; k < 3; ++k) {
			if (p[k] < mins[k])
				mins[k] = p[k];
			if (p[k] > maxs[k])
				maxs[k] = p[k];
		}
	}
	if (numVerts == 0) {
		mins[0] = mins[1] = mins[2] = 0;
		maxs[0] = maxs[1] = maxs[2] = 0;
	}
	float cx = 0.5f * (mins[0] + maxs[0]);
	float cy = 0.5f * (mins[1] + maxs[1]);
	float cz = 0.5f * (mins[2] + maxs[2]);
	float r2 = 0;
	for (int i = 0; i < numVerts; ++i) {
		const float *p = xyz + i * 3;
		float dx = p[0] - cx;
		float dy = p[1] - cy;
		float dz = p[2] - cz;
		float d2 = dx * dx + dy * dy + dz * dz;
		if (d2 > r2)
			r2 = d2;
	}
	if (outRadius)
		*outRadius = sqrtf(r2);
}

void mc_model_init(mc_model_t *m) {
	memset(m, 0, sizeof(*m));
	m->fps = 15.0f;
}

void mc_surface_alloc(mc_surface_t *s, int numVerts, int numTris, int numFrames) {
	s->numVerts = numVerts;
	s->numTris = numTris;
	if (numVerts > 0 && numFrames > 0) {
		size_t n = mc_safe_mul3((size_t)numVerts, (size_t)numFrames, 3);
		s->xyz = (float *)mc_calloc(n, sizeof(float));
		s->normal = (float *)mc_calloc(n, sizeof(float));
		s->st = (float *)mc_calloc((size_t)numVerts * 2, sizeof(float));
	}
	if (numTris > 0) {
		s->indices = (int *)mc_calloc((size_t)numTris * 3, sizeof(int));
	}
}

void mc_surface_alloc_blend(mc_surface_t *s) {
	if (s->numVerts <= 0) return;
	if (!s->blendIndices)
		s->blendIndices = (unsigned char *)mc_calloc((size_t)s->numVerts * 4, 1);
	if (!s->blendWeights)
		s->blendWeights = (float *)mc_calloc((size_t)s->numVerts * 4, sizeof(float));
}

void mc_model_set_joints(mc_model_t *m, int numJoints) {
	if (numJoints <= 0) {
		free(m->joints);
		free(m->jointPoses);
		m->joints = NULL;
		m->jointPoses = NULL;
		m->numJoints = 0;
		return;
	}
	mc_joint_t *nj = (mc_joint_t *)mc_calloc((size_t)numJoints, sizeof(mc_joint_t));
	if (m->joints && m->numJoints > 0) {
		int n = numJoints < m->numJoints ? numJoints : m->numJoints;
		memcpy(nj, m->joints, sizeof(mc_joint_t) * (size_t)n);
		free(m->joints);
	}
	m->joints = nj;
	int frames = m->numFrames > 0 ? m->numFrames : 1;
	mc_joint_pose_t *np = (mc_joint_pose_t *)mc_calloc((size_t)frames * (size_t)numJoints, sizeof(mc_joint_pose_t));
	free(m->jointPoses);
	m->jointPoses = np;
	m->numJoints = numJoints;
}

void mc_model_free(mc_model_t *m) {
	if (!m)
		return;
	free(m->frames);
	free(m->tags);
	for (int i = 0; i < m->numSurfaces; ++i) {
		mc_surface_t *s = &m->surfaces[i];
		free(s->xyz);
		free(s->normal);
		free(s->st);
		free(s->indices);
		free(s->blendIndices);
		free(s->blendWeights);
		free(s->q3_shader_body);
	}
	free(m->surfaces);
	free(m->animations);
	for (int i = 0; i < m->numSkins; ++i) {
		free(m->skins[i].entries);
	}
	free(m->skins);
	free(m->joints);
	free(m->jointPoses);
	memset(m, 0, sizeof(*m));
}

mc_skin_variant_t *mc_model_add_skin_variant(mc_model_t *m, const char *name, const char *part) {
	int n = m->numSkins + 1;
	mc_skin_variant_t *p = (mc_skin_variant_t *)mc_realloc(m->skins, sizeof(mc_skin_variant_t) * (size_t)n);
	if (!p)
		return NULL;
	m->skins = p;
	mc_skin_variant_t *v = &p[m->numSkins];
	memset(v, 0, sizeof(*v));
	if (name)
		mc_q_strncpy(v->name, name, sizeof(v->name));
	if (part)
		mc_q_strncpy(v->part, part, sizeof(v->part));
	m->numSkins = n;
	return v;
}

mc_animation_t *mc_model_add_animation(mc_model_t *m) {
	int n = m->numAnimations + 1;
	mc_animation_t *p = (mc_animation_t *)mc_realloc(m->animations, sizeof(mc_animation_t) * (size_t)n);
	if (!p)
		return NULL;
	m->animations = p;
	mc_animation_t *a = &p[m->numAnimations];
	memset(a, 0, sizeof(*a));
	m->numAnimations = n;
	return a;
}

mc_surface_t *mc_model_add_surface(mc_model_t *m) {
	int n = m->numSurfaces + 1;
	mc_surface_t *p = (mc_surface_t *)mc_realloc(m->surfaces, sizeof(mc_surface_t) * (size_t)n);
	if (!p)
		return NULL;
	m->surfaces = p;
	mc_surface_t *s = &p[m->numSurfaces];
	memset(s, 0, sizeof(*s));
	/* Sensible material defaults: opaque white, no alpha test. */
	s->base_color[0] = s->base_color[1] = s->base_color[2] = s->base_color[3] = 1.0f;
	s->alpha_cutoff = 0.5f;
	s->metallic_factor = 1.0f;
	s->roughness_factor = 1.0f;
	s->occlusion_strength = 1.0f;
	m->numSurfaces = n;
	return s;
}

mc_frame_t *mc_model_add_frame(mc_model_t *m) {
	int n = m->numFrames + 1;
	mc_frame_t *p = (mc_frame_t *)mc_realloc(m->frames, sizeof(mc_frame_t) * (size_t)n);
	if (!p)
		return NULL;
	m->frames = p;
	mc_frame_t *f = &p[m->numFrames];
	memset(f, 0, sizeof(*f));
	m->numFrames = n;
	return f;
}

/* Tags are stored as numFrames * numTags; this helper reallocates the array
   when adding a brand-new tag slot. Caller is expected to pass the *current*
   numFrames so we can lay out tag rows correctly afterwards. */
mc_tag_t *mc_model_add_tag_slot(mc_model_t *m, int numFrames) {
	int oldTags = m->numTags;
	int newTags = oldTags + 1;
	int rows = numFrames > 0 ? numFrames : 1;
	mc_tag_t *p = (mc_tag_t *)mc_calloc((size_t)rows * (size_t)newTags, sizeof(mc_tag_t));
	if (!p)
		return NULL;
	if (m->tags) {
		for (int f = 0; f < rows; ++f) {
			for (int t = 0; t < oldTags; ++t) {
				p[f * newTags + t] = m->tags[f * oldTags + t];
			}
		}
		free(m->tags);
	}
	m->tags = p;
	m->numTags = newTags;
	return &p[newTags - 1];
}

mc_format_t mc_guess_format(const char *path) {
	const char *dot = strrchr(path, '.');
	if (!dot)
		return MC_FMT_UNKNOWN;
	if (!strcasecmp(dot, ".gltf"))
		return MC_FMT_GLTF;
	if (!strcasecmp(dot, ".glb"))
		return MC_FMT_GLB;
	if (!strcasecmp(dot, ".md3"))
		return MC_FMT_MD3;
	if (!strcasecmp(dot, ".iqm"))
		return MC_FMT_IQM;
	if (!strcasecmp(dot, ".mdr"))
		return MC_FMT_MDR;
	return MC_FMT_UNKNOWN;
}

const char *mc_format_name(mc_format_t f) {
	switch (f) {
	case MC_FMT_GLTF:
		return "glTF";
	case MC_FMT_GLB:
		return "glb";
	case MC_FMT_MD3:
		return "md3";
	case MC_FMT_IQM:
		return "iqm";
	case MC_FMT_MDR:
		return "mdr";
	default:
		return "unknown";
	}
}

int mc_save_skin(const char *path, const mc_model_t *m) {
	FILE *f = fopen(path, "wb");
	if (!f) {
		MC_ERR("cannot open '%s' for writing\n", path);
		return -1;
	}
	for (int i = 0; i < m->numSurfaces; ++i) {
		const mc_surface_t *s = &m->surfaces[i];
		const char *shader = s->shader[0] ? s->shader : (s->texture[0] ? s->texture : "");
		fprintf(f, "%s,%s\n", s->name, shader);
	}
	for (int i = 0; i < m->numTags; ++i) {
		fprintf(f, "tag_%s,\n", m->tags[i].name);
	}
	fclose(f);
	return 0;
}


