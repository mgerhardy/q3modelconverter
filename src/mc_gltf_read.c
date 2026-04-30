/*
===========================================================================
modelconverter - glTF/GLB reader

Loads a glTF or GLB file via cgltf and bakes node transforms into the
generic mc_model_t representation that the MD3 / IQM writers consume.

Conventions
-----------
- Every glTF mesh primitive becomes one mc_surface_t.  The surface name is
  derived from the owning node name (or mesh name as fallback) and the
  primitive index.
- Nodes whose name starts with "tag_" (case-insensitive) and that carry no
  mesh become an MD3-style tag.  The node's local axes (forward = +X, left
  = +Y, up = +Z, matching idTech3 convention) are emitted as the tag axes.
- If the glTF contains animations, the first animation channel set is
  sampled at `fps_hint` Hz and one mc_frame_t is produced per sample;
  vertex positions and normals are pre-baked per frame by transforming the
  bind-pose vertices through the animated node world matrix.
- Materials -> shader names: the shader field is set to the texture path
  (without extension) of the baseColor texture, falling back to the
  material name.  Texture image files referenced via URI are copied next
  to the output; embedded data URIs are written out using the original
  MIME type extension (typically PNG or JPEG).
===========================================================================
*/

#define CGLTF_IMPLEMENTATION
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-Wmissing-prototypes"
#endif
#include "cgltf.h"
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include "mc_common.h"

#include <ctype.h>

/* ------------------------------------------------------------------ */
/* Math helpers                                                       */
/* ------------------------------------------------------------------ */

static void mat4_identity(float m[16]) {
	memset(m, 0, sizeof(float) * 16);
	m[0] = m[5] = m[10] = m[15] = 1.0f;
}

static void mat4_mul(const float a[16], const float b[16], float out[16]) {
	float r[16];
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			float s = 0;
			for (int k = 0; k < 4; ++k) {
				s += a[i * 4 + k] * b[k * 4 + j];
			}
			r[i * 4 + j] = s;
		}
	}
	memcpy(out, r, sizeof(r));
}

static void mat4_transform_point(const float m[16], const float p[3], float out[3]) {
	float x = m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12];
	float y = m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13];
	float z = m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14];
	out[0] = x;
	out[1] = y;
	out[2] = z;
}

static void mat4_transform_dir(const float m[16], const float p[3], float out[3]) {
	float x = m[0] * p[0] + m[4] * p[1] + m[8] * p[2];
	float y = m[1] * p[0] + m[5] * p[1] + m[9] * p[2];
	float z = m[2] * p[0] + m[6] * p[1] + m[10] * p[2];
	out[0] = x;
	out[1] = y;
	out[2] = z;
}

static void normalize3(float v[3]) {
	float l = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	if (l > 1e-8f) {
		v[0] /= l;
		v[1] /= l;
		v[2] /= l;
	}
}

/* ------------------------------------------------------------------ */
/* Quaternion + 3x4 matrix helpers used by the joint-animation sampler */
/* ------------------------------------------------------------------ */

static void quat_normalize(float q[4]) {
	float l = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if (l > 1e-8f) {
		q[0] /= l;
		q[1] /= l;
		q[2] /= l;
		q[3] /= l;
	} else {
		q[0] = q[1] = q[2] = 0.0f;
		q[3] = 1.0f;
	}
}

/* Spherical linear interpolation, xyzw quaternions; falls back to lerp for
   nearly antipodal pairs. */
static void quat_slerp(const float a[4], const float b[4], float t, float out[4]) {
	float bx = b[0], by = b[1], bz = b[2], bw = b[3];
	float cosT = a[0] * bx + a[1] * by + a[2] * bz + a[3] * bw;
	if (cosT < 0.0f) {
		bx = -bx; by = -by; bz = -bz; bw = -bw;
		cosT = -cosT;
	}
	float s0, s1;
	if (cosT > 0.9995f) {
		s0 = 1.0f - t;
		s1 = t;
	} else {
		float omega = acosf(cosT);
		float sinO = sinf(omega);
		s0 = sinf((1.0f - t) * omega) / sinO;
		s1 = sinf(t * omega) / sinO;
	}
	out[0] = s0 * a[0] + s1 * bx;
	out[1] = s0 * a[1] + s1 * by;
	out[2] = s0 * a[2] + s1 * bz;
	out[3] = s0 * a[3] + s1 * bw;
	quat_normalize(out);
}

static void mat34_identity(float m[3][4]) {
	memset(m, 0, sizeof(float) * 12);
	m[0][0] = m[1][1] = m[2][2] = 1.0f;
}

static void mat34_from_trs(const float t[3], const float q[4], const float s[3], float m[3][4]) {
	float x = q[0], y = q[1], z = q[2], w = q[3];
	float ql = sqrtf(x * x + y * y + z * z + w * w);
	if (ql > 1e-8f) { x /= ql; y /= ql; z /= ql; w /= ql; }
	float xx = x * x, yy = y * y, zz = z * z;
	float xy = x * y, xz = x * z, yz = y * z;
	float wx = w * x, wy = w * y, wz = w * z;
	m[0][0] = (1 - 2 * (yy + zz)) * s[0];
	m[0][1] = (2 * (xy - wz)) * s[1];
	m[0][2] = (2 * (xz + wy)) * s[2];
	m[0][3] = t[0];
	m[1][0] = (2 * (xy + wz)) * s[0];
	m[1][1] = (1 - 2 * (xx + zz)) * s[1];
	m[1][2] = (2 * (yz - wx)) * s[2];
	m[1][3] = t[1];
	m[2][0] = (2 * (xz - wy)) * s[0];
	m[2][1] = (2 * (yz + wx)) * s[1];
	m[2][2] = (1 - 2 * (xx + yy)) * s[2];
	m[2][3] = t[2];
}

static void mat34_mul(const float a[3][4], const float b[3][4], float out[3][4]) {
	float r[3][4];
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j)
			r[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
		r[i][3] = a[i][0] * b[0][3] + a[i][1] * b[1][3] + a[i][2] * b[2][3] + a[i][3];
	}
	memcpy(out, r, sizeof(r));
}

static void mat34_invert(const float m[3][4], float out[3][4]) {
	float a = m[0][0], b = m[0][1], c = m[0][2];
	float d = m[1][0], e = m[1][1], f = m[1][2];
	float g = m[2][0], h = m[2][1], i = m[2][2];
	float det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
	if (fabsf(det) < 1e-10f) { mat34_identity(out); return; }
	float inv = 1.0f / det;
	float r[3][3];
	r[0][0] = (e * i - f * h) * inv;
	r[0][1] = (c * h - b * i) * inv;
	r[0][2] = (b * f - c * e) * inv;
	r[1][0] = (f * g - d * i) * inv;
	r[1][1] = (a * i - c * g) * inv;
	r[1][2] = (c * d - a * f) * inv;
	r[2][0] = (d * h - e * g) * inv;
	r[2][1] = (b * g - a * h) * inv;
	r[2][2] = (a * e - b * d) * inv;
	out[0][0] = r[0][0]; out[0][1] = r[0][1]; out[0][2] = r[0][2];
	out[1][0] = r[1][0]; out[1][1] = r[1][1]; out[1][2] = r[1][2];
	out[2][0] = r[2][0]; out[2][1] = r[2][1]; out[2][2] = r[2][2];
	out[0][3] = -(r[0][0] * m[0][3] + r[0][1] * m[1][3] + r[0][2] * m[2][3]);
	out[1][3] = -(r[1][0] * m[0][3] + r[1][1] * m[1][3] + r[1][2] * m[2][3]);
	out[2][3] = -(r[2][0] * m[0][3] + r[2][1] * m[1][3] + r[2][2] * m[2][3]);
}

static void mat34_xform_point(const float m[3][4], const float p[3], float out[3]) {
	out[0] = m[0][0] * p[0] + m[0][1] * p[1] + m[0][2] * p[2] + m[0][3];
	out[1] = m[1][0] * p[0] + m[1][1] * p[1] + m[1][2] * p[2] + m[1][3];
	out[2] = m[2][0] * p[0] + m[2][1] * p[1] + m[2][2] * p[2] + m[2][3];
}

static void mat34_xform_dir(const float m[3][4], const float p[3], float out[3]) {
	out[0] = m[0][0] * p[0] + m[0][1] * p[1] + m[0][2] * p[2];
	out[1] = m[1][0] * p[0] + m[1][1] * p[1] + m[1][2] * p[2];
	out[2] = m[2][0] * p[0] + m[2][1] * p[1] + m[2][2] * p[2];
}

/* Evaluate a glTF animation sampler at time `time` and write `comp`
   floats into `out`.  Supports STEP, LINEAR, and CUBIC SPLINE
   interpolation modes.  Quaternion (comp==4) interpolation uses slerp
   for LINEAR; vec3 / scalar interpolations use plain lerp. */
static void sampler_evaluate(const cgltf_animation_sampler *samp, float time, int comp, float *out) {
	const cgltf_accessor *inAcc = samp->input;
	const cgltf_accessor *outAcc = samp->output;
	cgltf_size n = inAcc->count;
	if (n == 0) {
		memset(out, 0, sizeof(float) * (size_t)comp);
		if (comp == 4) out[3] = 1.0f;
		return;
	}
	float t0 = 0.0f, t1 = 0.0f;
	cgltf_accessor_read_float(inAcc, 0, &t0, 1);
	cgltf_accessor_read_float(inAcc, n - 1, &t1, 1);
	if (time <= t0) {
		cgltf_size base = (samp->interpolation == cgltf_interpolation_type_cubic_spline) ? 1 : 0;
		cgltf_accessor_read_float(outAcc, base, out, (cgltf_size)comp);
		return;
	}
	if (time >= t1) {
		cgltf_size last = (samp->interpolation == cgltf_interpolation_type_cubic_spline) ? (n - 1) * 3 + 1
																						 : (n - 1);
		cgltf_accessor_read_float(outAcc, last, out, (cgltf_size)comp);
		return;
	}
	/* Binary search for the surrounding key frames. */
	cgltf_size lo = 0, hi = n - 1;
	while (hi - lo > 1) {
		cgltf_size mid = (lo + hi) >> 1;
		float tm = 0.0f;
		cgltf_accessor_read_float(inAcc, mid, &tm, 1);
		if (tm <= time) lo = mid; else hi = mid;
	}
	float ta = 0.0f, tb = 0.0f;
	cgltf_accessor_read_float(inAcc, lo, &ta, 1);
	cgltf_accessor_read_float(inAcc, hi, &tb, 1);
	float dt = tb - ta;
	float u = dt > 1e-12f ? (time - ta) / dt : 0.0f;

	if (samp->interpolation == cgltf_interpolation_type_step) {
		cgltf_accessor_read_float(outAcc, lo, out, (cgltf_size)comp);
		return;
	}
	if (samp->interpolation == cgltf_interpolation_type_cubic_spline) {
		float v0[4], m0[4], v1[4], m1[4];
		cgltf_accessor_read_float(outAcc, lo * 3 + 1, v0, (cgltf_size)comp);
		cgltf_accessor_read_float(outAcc, lo * 3 + 2, m0, (cgltf_size)comp);
		cgltf_accessor_read_float(outAcc, hi * 3 + 0, m1, (cgltf_size)comp);
		cgltf_accessor_read_float(outAcc, hi * 3 + 1, v1, (cgltf_size)comp);
		float u2 = u * u;
		float u3 = u2 * u;
		float h00 = 2 * u3 - 3 * u2 + 1;
		float h10 = u3 - 2 * u2 + u;
		float h01 = -2 * u3 + 3 * u2;
		float h11 = u3 - u2;
		for (int c = 0; c < comp; ++c)
			out[c] = h00 * v0[c] + h10 * dt * m0[c] + h01 * v1[c] + h11 * dt * m1[c];
		if (comp == 4) quat_normalize(out);
		return;
	}
	/* LINEAR (default). */
	float a[4] = {0, 0, 0, 0};
	float b[4] = {0, 0, 0, 0};
	cgltf_accessor_read_float(outAcc, lo, a, (cgltf_size)comp);
	cgltf_accessor_read_float(outAcc, hi, b, (cgltf_size)comp);
	if (comp == 4) {
		quat_slerp(a, b, u, out);
	} else {
		for (int c = 0; c < comp; ++c)
			out[c] = a[c] + (b[c] - a[c]) * u;
	}
}

/* Compose a node's world matrix while skipping any ancestor that the
   writer marked as `q3_pivot` (the lower/upper/head joint nodes used
   only for visual alignment in glTF viewers).  Vertices and tags
   stored under those pivots remain in part-local space, so we drop the
   pivot transforms when reading them back into the in-memory model. */
static int extras_get_string(const char *json, const char *key, char *out, size_t out_size);
static void q3_compose_world(const cgltf_node *node, float out[16]) {
	mat4_identity(out);
	const cgltf_node *chain[64];
	int depth = 0;
	for (const cgltf_node *n = node; n && depth < 64; n = n->parent)
		chain[depth++] = n;
	for (int i = depth - 1; i >= 0; --i) {
		const cgltf_node *n = chain[i];
		int is_pivot = 0;
		if (n->extras.data && n->extras.data[0]) {
			char tmp[32];
			if (extras_get_string(n->extras.data, "q3_pivot", tmp, sizeof(tmp)))
				is_pivot = 1;
		}
		if (is_pivot)
			continue;
		float local[16];
		cgltf_node_transform_local(n, local);
		mat4_mul(out, local, out);
	}
}

/* ------------------------------------------------------------------ */
/* Texture / material extraction                                      */
/* ------------------------------------------------------------------ */

/* Probe for a `_n` / `_nh` / `_d` companion next to the base color
   image's source URI, and if it exists, copy it next to base_dir so the
   produced model can find it.  The copy mirrors emit_texture's "flatten
   to basename" rule so the resulting `.shader` keeps consistent paths.
   Only handles external (file) URIs; embedded glb-internal images have
   no on-disk siblings to discover. */
static int companion_probe(const cgltf_image *base_img, const char *suffix, const char *gltf_dir, const char *base_dir,
						   char *out, size_t out_size) {
	out[0] = 0;
	if (!base_img || !base_img->uri || !suffix || !suffix[0])
		return 0;
	const char *uri = base_img->uri;
	if (!strncmp(uri, "data:", 5))
		return 0;

	/* Split URI into "<dir>/<stem>.<ext>". */
	const char *slash = strrchr(uri, '/');
	const char *fname = slash ? slash + 1 : uri;
	size_t dir_len = slash ? (size_t)(slash - uri + 1) : 0;
	const char *dot = strrchr(fname, '.');
	size_t stem_len = dot ? (size_t)(dot - fname) : strlen(fname);

	static const char *exts[] = {".png", ".tga", ".jpg", ".jpeg", ".webp", NULL};
	const char *orig_ext_with_dot = dot ? dot : "";
	for (int e = -1; exts[e + 1]; ++e) {
		/* Try the original extension first, then the fallback list. */
		const char *try_ext = (e < 0) ? orig_ext_with_dot : exts[e];
		if (!try_ext[0])
			continue;
		char rel[MC_MAX_PATH];
		if (dir_len)
			snprintf(rel, sizeof(rel), "%.*s%.*s%s%s", (int)dir_len, uri, (int)stem_len, fname, suffix, try_ext);
		else
			snprintf(rel, sizeof(rel), "%.*s%s%s", (int)stem_len, fname, suffix, try_ext);

		char src[MC_MAX_PATH];
		if (gltf_dir && gltf_dir[0])
			mc_join_path(src, sizeof(src), gltf_dir, rel);
		else
			mc_q_strncpy(src, rel, sizeof(src));

		size_t sz = 0;
		unsigned char *data = mc_read_file_quiet(src, &sz);
		if (!data)
			continue;
		/* Mirror emit_texture: copy as basename into base_dir. */
		const char *base_name = mc_basename(rel);
		if (base_dir && base_dir[0]) {
			char dst[MC_MAX_PATH];
			mc_join_path(dst, sizeof(dst), base_dir, base_name);
			mc_write_file(dst, data, sz);
		}
		free(data);
		mc_q_strncpy(out, base_name, out_size);
		return 1;
	}
	return 0;
}

static const char *mime_extension(const char *mime) {
	if (!mime)
		return "png";
	if (strstr(mime, "png"))
		return "png";
	if (strstr(mime, "jpeg") || strstr(mime, "jpg"))
		return "jpg";
	if (strstr(mime, "ktx"))
		return "ktx";
	if (strstr(mime, "webp"))
		return "webp";
	return "bin";
}

/* base64 decoder (RFC 4648) used to extract embedded data URIs. */
static int b64_value(int c) {
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

static unsigned char *b64_decode(const char *in, size_t in_len, size_t *out_len) {
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
		int v = b64_value(c);
		if (v < 0)
			continue;
		val = (val << 6) | v;
		bits += 6;
		if (bits >= 8) {
			bits -= 8;
			out[o++] = (unsigned char)((val >> bits) & 0xff);
		}
	}
	*out_len = o;
	return out;
}

static void emit_texture(const cgltf_image *img, const char *base_dir, const char *gltf_dir, char *out_relpath,
						 size_t out_len) {
	out_relpath[0] = 0;
	if (!img)
		return;

	char fileName[MC_MAX_PATH];
	if (img->name && img->name[0]) {
		mc_q_strncpy(fileName, img->name, sizeof(fileName));
		mc_strip_extension(fileName);
	} else {
		snprintf(fileName, sizeof(fileName), "tex_%p", (const void *)img);
	}

	if (img->uri) {
		const char *uri = img->uri;
		if (!strncmp(uri, "data:", 5)) {
			/* data:[<mime>][;base64],<payload> */
			const char *comma = strchr(uri, ',');
			if (!comma)
				return;
			char mime[64] = "image/png";
			const char *header = uri + 5;
			size_t hlen = (size_t)(comma - header);
			size_t copy = hlen < sizeof(mime) - 1 ? hlen : sizeof(mime) - 1;
			memcpy(mime, header, copy);
			mime[copy] = 0;
			int isB64 = strstr(mime, "base64") != NULL;
			char *semi = strchr(mime, ';');
			if (semi)
				*semi = 0;
			const char *ext = mime_extension(mime);

			char outRel[MC_MAX_PATH];
			snprintf(outRel, sizeof(outRel), "%s.%s", fileName, ext);
			char outPath[MC_MAX_PATH];
			mc_join_path(outPath, sizeof(outPath), base_dir, outRel);

			if (isB64) {
				size_t outSize = 0;
				unsigned char *bin = b64_decode(comma + 1, strlen(comma + 1), &outSize);
				if (bin) {
					mc_write_file(outPath, bin, outSize);
					free(bin);
				}
			} else {
				/* URL-encoded raw payload (RFC 3986 percent-encoding).
				   Decode %xx escapes and '+' (space) before writing,
				   so an embedded PNG/JPEG round-trips byte-identical
				   instead of leaking the URI escape sequences into
				   the on-disk file. */
				const char *src = comma + 1;
				size_t srclen = strlen(src);
				unsigned char *bin = (unsigned char *)mc_malloc(srclen + 1);
				if (bin) {
					size_t w = 0;
					for (size_t r = 0; r < srclen; ++r) {
						unsigned char c = (unsigned char)src[r];
						if (c == '%' && r + 2 < srclen) {
							char hex[3] = { src[r + 1], src[r + 2], 0 };
							char *end = NULL;
							unsigned long v = strtoul(hex, &end, 16);
							if (end == hex + 2) {
								bin[w++] = (unsigned char)v;
								r += 2;
								continue;
							}
						}
						if (c == '+') {
							bin[w++] = ' ';
							continue;
						}
						bin[w++] = c;
					}
					mc_write_file(outPath, bin, w);
					free(bin);
				}
			}
			mc_q_strncpy(out_relpath, outRel, out_len);
		} else {
			/* External file - copy from the gltf's directory. */
			char src[MC_MAX_PATH];
			mc_join_path(src, sizeof(src), gltf_dir, uri);
			size_t sz = 0;
			unsigned char *data = mc_read_file(src, &sz);
			if (!data)
				return;
			char outPath[MC_MAX_PATH];
			mc_join_path(outPath, sizeof(outPath), base_dir, mc_basename(uri));
			mc_write_file(outPath, data, sz);
			free(data);
			mc_q_strncpy(out_relpath, mc_basename(uri), out_len);
		}
	} else if (img->buffer_view && img->buffer_view->buffer && img->buffer_view->buffer->data) {
		const char *ext = mime_extension(img->mime_type);
		char outRel[MC_MAX_PATH];
		snprintf(outRel, sizeof(outRel), "%s.%s", fileName, ext);
		char outPath[MC_MAX_PATH];
		mc_join_path(outPath, sizeof(outPath), base_dir, outRel);
		const unsigned char *src = (const unsigned char *)img->buffer_view->buffer->data;
		mc_write_file(outPath, src + img->buffer_view->offset, img->buffer_view->size);
		mc_q_strncpy(out_relpath, outRel, out_len);
	}
}

static void resolve_material(const cgltf_material *mat, const cgltf_data *data, const char *base_dir,
							 const char *gltf_dir, mc_surface_t *out) {
	out->shader[0] = 0;
	out->texture[0] = 0;
	out->normal_map[0] = 0;
	out->normal_height_map[0] = 0;
	out->emissive_map[0] = 0;
	out->mr_map[0] = 0;
	out->occlusion_map[0] = 0;
	out->surfaceparm[0] = 0;
	out->two_sided = 0;
	out->alpha_mode = 0;
	out->alpha_cutoff = 0.5f;
	out->base_color[0] = out->base_color[1] = out->base_color[2] = out->base_color[3] = 1.0f;
	out->emissive_factor[0] = out->emissive_factor[1] = out->emissive_factor[2] = 0.0f;
	out->metallic_factor = 1.0f;
	out->roughness_factor = 1.0f;
	out->occlusion_strength = 1.0f;
	if (!mat)
		return;

	const cgltf_texture *tex = NULL;
	if (mat->has_pbr_metallic_roughness && mat->pbr_metallic_roughness.base_color_texture.texture) {
		tex = mat->pbr_metallic_roughness.base_color_texture.texture;
	} else if (mat->emissive_texture.texture) {
		tex = mat->emissive_texture.texture;
	}

	if (tex && tex->image && base_dir) {
		emit_texture(tex->image, base_dir, gltf_dir, out->texture, sizeof(out->texture));
	}
	if (mat->normal_texture.texture && mat->normal_texture.texture->image && base_dir) {
		emit_texture(mat->normal_texture.texture->image, base_dir, gltf_dir, out->normal_map, sizeof(out->normal_map));
	}
	if (mat->emissive_texture.texture && mat->emissive_texture.texture->image && base_dir) {
		emit_texture(mat->emissive_texture.texture->image, base_dir, gltf_dir, out->emissive_map,
					 sizeof(out->emissive_map));
	}
	if (mat->has_pbr_metallic_roughness && mat->pbr_metallic_roughness.metallic_roughness_texture.texture &&
		mat->pbr_metallic_roughness.metallic_roughness_texture.texture->image && base_dir) {
		emit_texture(mat->pbr_metallic_roughness.metallic_roughness_texture.texture->image, base_dir, gltf_dir,
					 out->mr_map, sizeof(out->mr_map));
	}
	if (mat->occlusion_texture.texture && mat->occlusion_texture.texture->image && base_dir) {
		emit_texture(mat->occlusion_texture.texture->image, base_dir, gltf_dir,
					 out->occlusion_map, sizeof(out->occlusion_map));
		out->occlusion_strength = mat->occlusion_texture.scale;
	}

	if (mat->has_pbr_metallic_roughness) {
		for (int k = 0; k < 4; ++k)
			out->base_color[k] = mat->pbr_metallic_roughness.base_color_factor[k];
		out->metallic_factor = mat->pbr_metallic_roughness.metallic_factor;
		out->roughness_factor = mat->pbr_metallic_roughness.roughness_factor;
	}
	for (int k = 0; k < 3; ++k)
		out->emissive_factor[k] = mat->emissive_factor[k];

	out->two_sided = mat->double_sided ? 1 : 0;
	switch (mat->alpha_mode) {
	case cgltf_alpha_mode_mask:
		out->alpha_mode = 1;
		out->alpha_cutoff = mat->alpha_cutoff;
		break;
	case cgltf_alpha_mode_blend:
		out->alpha_mode = 2;
		break;
	default:
		out->alpha_mode = 0;
		break;
	}

	/* Build a Quake3 shader path: textures/<material name> if a name exists,
	   otherwise strip the texture extension. */
	if (mat->name && mat->name[0]) {
		mc_q_strncpy(out->shader, mat->name, sizeof(out->shader));
	} else if (out->texture[0]) {
		char tmp[MC_MAX_PATH];
		mc_q_strncpy(tmp, out->texture, sizeof(tmp));
		mc_strip_extension(tmp);
		mc_q_strncpy(out->shader, tmp, sizeof(out->shader));
	}

	/* Extras may carry our q3_shader override from a previous round-trip,
	   plus q3_normal / q3_normalheight / q3_deluxe paths we stamped on the
	   way out. */
	if (mat->extras.data && mat->extras.data[0]) {
		struct {
			const char *key;
			char *dst;
			size_t cap;
		} entries[] = {
			{"\"q3_shader\"", out->shader, sizeof(out->shader)},
			{"\"q3_normal\"", out->normal_map, sizeof(out->normal_map)},
			{"\"q3_normalheight\"", out->normal_height_map, sizeof(out->normal_height_map)},
		};
		for (int i = 0; i < (int)(sizeof(entries) / sizeof(entries[0])); ++i) {
			const char *ex = strstr(mat->extras.data, entries[i].key);
			if (!ex)
				continue;
			ex = strchr(ex + strlen(entries[i].key), '"');
			if (!ex)
				continue;
			++ex;
			const char *end = strchr(ex, '"');
			if (end && (size_t)(end - ex) < entries[i].cap) {
				memcpy(entries[i].dst, ex, (size_t)(end - ex));
				entries[i].dst[end - ex] = 0;
			}
		}

		/* q3_body_b64: base64-encoded raw shader stanza body captured on
		   the previous load. Decode and store so the next .shader write
		   can re-emit it verbatim. */
		const char *bex = strstr(mat->extras.data, "\"q3_body_b64\"");
		if (bex) {
			bex = strchr(bex + strlen("\"q3_body_b64\""), '"');
			if (bex) {
				++bex;
				const char *bend = strchr(bex, '"');
				if (bend && bend > bex) {
					size_t dec_len = 0;
					unsigned char *dec = mc_b64_decode(bex, (size_t)(bend - bex), &dec_len);
					if (dec) {
						free(out->q3_shader_body);
						out->q3_shader_body = (char *)mc_malloc(dec_len + 1);
						if (out->q3_shader_body) {
							memcpy(out->q3_shader_body, dec, dec_len);
							out->q3_shader_body[dec_len] = 0;
						}
						free(dec);
					}
				}
			}
		}
	}

	/* Auto-discover companion textures next to the base color image:
	   <name>_n.<ext>  -> normal map
	   <name>_nh.<ext> -> normal+height
	   The reader only fills slots that were not already populated by the
	   glTF material itself or extras above. */
	if (tex && tex->image) {
		if (!out->normal_map[0])
			companion_probe(tex->image, "_n", gltf_dir, base_dir, out->normal_map, sizeof(out->normal_map));
		if (!out->normal_height_map[0])
			companion_probe(tex->image, "_nh", gltf_dir, base_dir, out->normal_height_map,
							sizeof(out->normal_height_map));
	}
	(void)data;
}

/* ------------------------------------------------------------------ */
/* Accessor reads                                                     */
/* ------------------------------------------------------------------ */

static int read_floats(const cgltf_accessor *acc, float *out, int comp) {
	if (!acc)
		return 0;
	cgltf_size n = acc->count;
	for (cgltf_size i = 0; i < n; ++i) {
		cgltf_accessor_read_float(acc, i, out + i * comp, (cgltf_size)comp);
	}
	return (int)n;
}

static int read_indices(const cgltf_accessor *acc, int *out) {
	if (!acc)
		return 0;
	for (cgltf_size i = 0; i < acc->count; ++i) {
		out[i] = (int)cgltf_accessor_read_index(acc, i);
	}
	return (int)acc->count;
}

/* ------------------------------------------------------------------ */
/* Tag detection                                                      */
/* ------------------------------------------------------------------ */

static int is_tag_node(const cgltf_node *n) {
	if (!n || !n->name)
		return 0;
	if (n->mesh)
		return 0;
	const char *name = n->name;
	return (tolower((unsigned char)name[0]) == 't' && tolower((unsigned char)name[1]) == 'a' &&
			tolower((unsigned char)name[2]) == 'g' && name[3] == '_');
}

/* ------------------------------------------------------------------ */
/* Convention-based fallbacks                                         */
/* When q3_* extras are absent we try to recover the same metadata    */
/* from glTF naming conventions so files authored in Blender / etc.   */
/* without our extras still produce well-formed Q3 output.            */
/* ------------------------------------------------------------------ */

/* Return one of "head"/"upper"/"lower" if `name` matches that part by
   convention, else empty string.  Recognises:
     - exact match ("head", "upper", "lower")
     - h_/u_/l_ MD3 surface-name prefix
     - leading "head_", "upper_", "lower_"
     - leading "h_", "u_", "l_"
*/
static void infer_part_from_name(const char *name, char *out, size_t out_size) {
	out[0] = 0;
	if (!name || !name[0]) return;
	char buf[128];
	size_t i = 0;
	for (; name[i] && i + 1 < sizeof(buf); ++i) buf[i] = (char)tolower((unsigned char)name[i]);
	buf[i] = 0;
	const char *part = NULL;
	if (!strcmp(buf, "head") || !strncmp(buf, "head_", 5) || !strncmp(buf, "h_", 2))
		part = "head";
	else if (!strcmp(buf, "upper") || !strncmp(buf, "upper_", 6) || !strncmp(buf, "u_", 2)
			 || !strcmp(buf, "torso") || !strncmp(buf, "torso_", 6))
		part = "upper";
	else if (!strcmp(buf, "lower") || !strncmp(buf, "lower_", 6) || !strncmp(buf, "l_", 2)
			 || !strcmp(buf, "legs") || !strncmp(buf, "legs_", 5))
		part = "lower";
	if (part) mc_q_strncpy(out, part, out_size);
}

/* Walk up the glTF parent chain looking for a node whose name implies a
   Q3 player part.  This handles the common Blender layout
       Scene > head/ (empty) > head_mesh
   where the part is encoded in the parent group rather than the mesh
   primitive name. */
static void infer_part_from_node(const cgltf_node *node, char *out, size_t out_size) {
	out[0] = 0;
	for (const cgltf_node *n = node; n; n = n->parent) {
		if (!n->name) continue;
		infer_part_from_name(n->name, out, out_size);
		if (out[0]) return;
	}
	if (node && node->mesh && node->mesh->name)
		infer_part_from_name(node->mesh->name, out, out_size);
}

/* Recognise LOD suffix in a name.  Accepts trailing
       _lod1 / _lod2 / _LOD1 / .lod1 / -lod1
       _1 / _2 (only when value is 1..MD3_MAX_LODS-1)
   Returns the LOD index (0 if not recognised). */
static int infer_lod_from_name(const char *name) {
	if (!name) return 0;
	size_t n = strlen(name);
	if (n < 2) return 0;
	/* Look for "lod<digit>" anywhere near the tail. */
	for (size_t i = 0; i + 4 <= n; ++i) {
		if ((name[i] == 'l' || name[i] == 'L') && (name[i+1] == 'o' || name[i+1] == 'O')
			&& (name[i+2] == 'd' || name[i+2] == 'D')
			&& name[i+3] >= '0' && name[i+3] <= '9') {
			int v = name[i+3] - '0';
			if (v > 0 && v < MD3_MAX_LODS) return v;
		}
	}
	/* Trailing _<digit>. */
	if (name[n-2] == '_' && name[n-1] >= '1' && name[n-1] <= '9') {
		int v = name[n-1] - '0';
		if (v > 0 && v < MD3_MAX_LODS) return v;
	}
	return 0;
}

static int infer_lod_from_node(const cgltf_node *node) {
	for (const cgltf_node *n = node; n; n = n->parent) {
		if (!n->name) continue;
		int v = infer_lod_from_name(n->name);
		if (v) return v;
	}
	if (node && node->mesh && node->mesh->name)
		return infer_lod_from_name(node->mesh->name);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Extras parsing helpers                                              */
/* ------------------------------------------------------------------ */

/* Locate "key" in a small JSON blob and copy the associated string
   value (without surrounding quotes) into `out`.  Returns 1 on hit. */
static int extras_get_string(const char *json, const char *key, char *out, size_t out_size) {
	if (!json || !key)
		return 0;
	char needle[64];
	snprintf(needle, sizeof(needle), "\"%s\"", key);
	const char *p = strstr(json, needle);
	if (!p)
		return 0;
	p = strchr(p + strlen(needle), ':');
	if (!p)
		return 0;
	while (*p && (*p == ':' || isspace((unsigned char)*p)))
		++p;
	if (*p != '"')
		return 0;
	++p;
	size_t i = 0;
	while (*p && *p != '"' && i + 1 < out_size)
		out[i++] = *p++;
	out[i] = 0;
	return 1;
}

static int extras_has_true(const char *json, const char *key) {
	if (!json || !key)
		return 0;
	char needle[64];
	snprintf(needle, sizeof(needle), "\"%s\"", key);
	const char *p = strstr(json, needle);
	if (!p)
		return 0;
	p = strchr(p + strlen(needle), ':');
	if (!p)
		return 0;
	while (*p && (*p == ':' || isspace((unsigned char)*p)))
		++p;
	return strncmp(p, "true", 4) == 0;
}

/* Pull "key":[a,b,c] floats. */
static int extras_get_float_array(const char *json, const char *key, float *out, int max_count) {
	if (!json || !key)
		return 0;
	char needle[64];
	snprintf(needle, sizeof(needle), "\"%s\"", key);
	const char *p = strstr(json, needle);
	if (!p)
		return 0;
	p = strchr(p + strlen(needle), '[');
	if (!p)
		return 0;
	++p;
	int n = 0;
	while (*p && *p != ']' && n < max_count) {
		while (*p && (*p == ',' || isspace((unsigned char)*p)))
			++p;
		if (*p == ']')
			break;
		char *end = NULL;
		float v = strtof(p, &end);
		if (end == p)
			break;
		out[n++] = v;
		p = end;
	}
	return n;
}

/* Pull "key":[i,j,k] ints. */
static int extras_get_int_array(const char *json, const char *key, int *out, int max_count) {
	float tmp[16];
	if (max_count > 16)
		max_count = 16;
	int n = extras_get_float_array(json, key, tmp, max_count);
	for (int i = 0; i < n; ++i)
		out[i] = (int)tmp[i];
	return n;
}

/* Pull a scalar "key":<int>. Returns 1 if present, 0 otherwise. */
static int extras_get_int(const char *json, const char *key, int *out) {
	if (!json || !key)
		return 0;
	char needle[64];
	snprintf(needle, sizeof(needle), "\"%s\"", key);
	const char *p = strstr(json, needle);
	if (!p)
		return 0;
	p = strchr(p + strlen(needle), ':');
	if (!p)
		return 0;
	++p;
	while (*p && isspace((unsigned char)*p))
		++p;
	char *end = NULL;
	long v = strtol(p, &end, 10);
	if (end == p)
		return 0;
	*out = (int)v;
	return 1;
}

/* Parse the q3_animations array out of asset extras.  Each entry looks
   like {"name":"TORSO_GESTURE","first":N,"num":N,"loop":N,"fps":F}.
   Appends one mc_animation_t per entry. */
static void extras_parse_animations(const char *json, mc_model_t *m) {
	if (!json)
		return;
	const char *p = strstr(json, "\"q3_animations\"");
	if (!p)
		return;
	p = strchr(p + 15, '[');
	if (!p)
		return;
	++p;
	while (*p) {
		while (*p && (*p == ',' || isspace((unsigned char)*p)))
			++p;
		if (*p == ']' || *p == 0)
			break;
		const char *obj = strchr(p, '{');
		if (!obj)
			break;
		const char *objEnd = strchr(obj, '}');
		if (!objEnd)
			break;
		size_t len = (size_t)(objEnd - obj + 1);
		char buf[512];
		if (len >= sizeof(buf))
			len = sizeof(buf) - 1;
		memcpy(buf, obj, len);
		buf[len] = 0;

		mc_animation_t *a = mc_model_add_animation(m);
		if (!a)
			break;
		extras_get_string(buf, "name", a->name, sizeof(a->name));
		float tmp[1];
		if (extras_get_float_array(buf, "first", tmp, 1) || sscanf(strstr(buf, "\"first\"") ? strstr(buf, "\"first\"") + 7 : "", "%*[^0-9-]%d", &a->firstFrame) >= 0) {
			/* Read scalars via simple fallback: strstr+sscanf. */
		}
		const char *q;
		if ((q = strstr(buf, "\"first\"")) != NULL)
			sscanf(q + 7, "%*[^0-9-]%d", &a->firstFrame);
		if ((q = strstr(buf, "\"num\"")) != NULL)
			sscanf(q + 5, "%*[^0-9-]%d", &a->numFrames);
		if ((q = strstr(buf, "\"loop\"")) != NULL)
			sscanf(q + 6, "%*[^0-9-]%d", &a->loopFrames);
		if ((q = strstr(buf, "\"fps\"")) != NULL)
			sscanf(q + 5, "%*[^0-9.eE+-]%f", &a->fps);
		if (a->fps <= 0)
			a->fps = 15.0f;

		p = objEnd + 1;
	}
}

/* Parse q3_skins:[{"name":"...","entries":[{"surface":"...","material":"..."},...]},...]. */
static void extras_parse_skins(const char *json, mc_model_t *m) {
	if (!json)
		return;
	const char *p = strstr(json, "\"q3_skins\"");
	if (!p)
		return;
	p = strchr(p + 10, '[');
	if (!p)
		return;
	++p;
	int depth = 0;
	while (*p) {
		while (*p && (*p == ',' || isspace((unsigned char)*p)))
			++p;
		if (*p == ']' || *p == 0)
			break;
		if (*p != '{') {
			++p;
			continue;
		}
		/* Find matching closing brace for this skin object (handles nested entries[]). */
		const char *obj = p;
		depth = 0;
		const char *q;
		for (q = p; *q; ++q) {
			if (*q == '{')
				++depth;
			else if (*q == '}') {
				if (--depth == 0) {
					++q;
					break;
				}
			}
		}
		if (!*q && depth != 0)
			break;
		size_t len = (size_t)(q - obj);

		char name[64] = {0};
		char part[16] = {0};
		extras_get_string(obj, "name", name, sizeof(name));
		extras_get_string(obj, "part", part, sizeof(part));
		mc_skin_variant_t *sv = mc_model_add_skin_variant(m, name, part);
		if (sv) {
			const char *e = strstr(obj, "\"entries\"");
			if (e && (size_t)(e - obj) < len) {
				e = strchr(e + 9, '[');
				if (e) {
					++e;
					while (*e && e < q) {
						while (*e && (*e == ',' || isspace((unsigned char)*e)))
							++e;
						if (*e == ']' || e >= q)
							break;
						if (*e != '{') {
							++e;
							continue;
						}
						const char *eEnd = strchr(e, '}');
						if (!eEnd || eEnd >= q)
							break;
						char ebuf[256];
						size_t elen = (size_t)(eEnd - e + 1);
						if (elen >= sizeof(ebuf))
							elen = sizeof(ebuf) - 1;
						memcpy(ebuf, e, elen);
						ebuf[elen] = 0;
						char surf[MC_MAX_QPATH] = {0};
						char shd[MC_MAX_QPATH] = {0};
						extras_get_string(ebuf, "surface", surf, sizeof(surf));
						extras_get_string(ebuf, "shader", shd, sizeof(shd));
						mc_skin_entry_t *new_entries = (mc_skin_entry_t *)mc_realloc(
							sv->entries, sizeof(mc_skin_entry_t) * (size_t)(sv->numEntries + 1));
						if (new_entries) {
							sv->entries = new_entries;
							mc_skin_entry_t *en = &sv->entries[sv->numEntries++];
							memset(en, 0, sizeof(*en));
							mc_q_strncpy(en->surface, surf, sizeof(en->surface));
							mc_q_strncpy(en->shader, shd, sizeof(en->shader));
						}
						e = eEnd + 1;
					}
				}
			}
		}
		p = q;
	}
}

/* ------------------------------------------------------------------ */
/* Main loader                                                        */
/* ------------------------------------------------------------------ */

int mc_load_gltf(const char *path, mc_model_t *out, float fps_hint) {
	cgltf_options opts = {0};
	cgltf_data *data = NULL;
	cgltf_result r = cgltf_parse_file(&opts, path, &data);
	if (r != cgltf_result_success) {
		MC_ERR("gltf: parse failed (%d)\n", (int)r);
		return -1;
	}
	r = cgltf_load_buffers(&opts, data, path);
	if (r != cgltf_result_success) {
		MC_ERR("gltf: load_buffers failed (%d)\n", (int)r);
		cgltf_free(data);
		return -1;
	}

	mc_model_init(out);
	out->fps = fps_hint > 0 ? fps_hint : 15.0f;

	/* Asset extras: q3_player flag, q3_part_frames, q3_sex, q3_headoffset,
	   q3_animations.  Restored before any per-surface processing so the
	   downstream MD3 / player splitter sees them. */
	if (data->asset.extras.data && data->asset.extras.data[0]) {
		const char *aj = data->asset.extras.data;
		if (extras_has_true(aj, "q3_player"))
			out->is_player_bundle = 1;
		int pf[3] = {0, 0, 0};
		if (extras_get_int_array(aj, "q3_part_frames", pf, 3) > 0) {
			out->part_numFrames[0] = pf[0];
			out->part_numFrames[1] = pf[1];
			out->part_numFrames[2] = pf[2];
		}
		char sex[8];
		if (extras_get_string(aj, "q3_sex", sex, sizeof(sex)) && sex[0])
			out->anim_sex = sex[0];
		float ho[3] = {0, 0, 0};
		if (extras_get_float_array(aj, "q3_headoffset", ho, 3) == 3) {
			out->anim_headoffset[0] = ho[0];
			out->anim_headoffset[1] = ho[1];
			out->anim_headoffset[2] = ho[2];
			out->anim_has_headoffset = 1;
		}
		extras_parse_animations(aj, out);
		extras_parse_skins(aj, out);
	}

	char gltf_dir[MC_MAX_PATH];
	mc_dirname(path, gltf_dir, sizeof(gltf_dir));

	char base_dir[MC_MAX_PATH];
	mc_q_strncpy(base_dir, gltf_dir, sizeof(base_dir));

	const cgltf_scene *scene = data->scene ? data->scene : (data->scenes_count ? &data->scenes[0] : NULL);
	if (!scene) {
		MC_ERR("gltf: no scene\n");
		cgltf_free(data);
		return -1;
	}

	/* Pass 1: figure out how many output frames we'll need.  We honour
	   the largest morph-target count across primitives so models written
	   by us round-trip exactly. */
	int morphFrames = 1;
	for (cgltf_size i = 0; i < data->nodes_count; ++i) {
		const cgltf_node *node = &data->nodes[i];
		if (!node->mesh)
			continue;
		const cgltf_mesh *mesh = node->mesh;
		for (cgltf_size pi = 0; pi < mesh->primitives_count; ++pi) {
			const cgltf_primitive *prim = &mesh->primitives[pi];
			int frames = 1 + (int)prim->targets_count;
			if (frames > morphFrames)
				morphFrames = frames;
		}
	}

	/* ---- Skeleton: build the global joint table up front so we can
	   detect skeletal animations in the next step. */
	int *gltfNode_to_joint = NULL;
	if (data->nodes_count > 0) {
		gltfNode_to_joint = (int *)mc_malloc(sizeof(int) * data->nodes_count);
		for (cgltf_size i = 0; i < data->nodes_count; ++i)
			gltfNode_to_joint[i] = -1;
	}
	int totalJoints = 0;
	for (cgltf_size si = 0; si < data->skins_count; ++si) {
		const cgltf_skin *sk = &data->skins[si];
		for (cgltf_size j = 0; j < sk->joints_count; ++j) {
			cgltf_size idx = (cgltf_size)(sk->joints[j] - data->nodes);
			if (idx < data->nodes_count && gltfNode_to_joint[idx] < 0)
				gltfNode_to_joint[idx] = totalJoints++;
		}
	}

	/* ---- Detect skeletal-animation mode.  Treat the file as skeletal
	   when any cgltf_animation channel targets a registered joint
	   node's TRS - in that case we sample the channel stream into per
	   frame jointPoses and re-bake the surfaces' xyz / normal arrays
	   below.  When no joint channels are present we keep the legacy
	   morph-target path so files written by this tool round-trip
	   losslessly. */
	int skeletalMode = 0;
	int *animFrames = NULL;   /* per anim: number of sampled frames */
	float *animTimeMin = NULL;
	float *animTimeMax = NULL;
	int totalAnimFrames = 0;
	if (totalJoints > 0 && data->animations_count > 0) {
		for (cgltf_size ai = 0; ai < data->animations_count && !skeletalMode; ++ai) {
			const cgltf_animation *anim = &data->animations[ai];
			for (cgltf_size ci = 0; ci < anim->channels_count; ++ci) {
				const cgltf_animation_channel *ch = &anim->channels[ci];
				if (!ch->target_node) continue;
				cgltf_size nidx = (cgltf_size)(ch->target_node - data->nodes);
				if (nidx >= data->nodes_count) continue;
				if (gltfNode_to_joint[nidx] < 0) continue;
				if (ch->target_path != cgltf_animation_path_type_translation
					&& ch->target_path != cgltf_animation_path_type_rotation
					&& ch->target_path != cgltf_animation_path_type_scale)
					continue;
				skeletalMode = 1;
				break;
			}
		}
	}
	if (data->animations_count > 0) {
		float fps = fps_hint > 0.0f ? fps_hint : 15.0f;
		animFrames = (int *)mc_calloc((size_t)data->animations_count, sizeof(int));
		animTimeMin = (float *)mc_calloc((size_t)data->animations_count, sizeof(float));
		animTimeMax = (float *)mc_calloc((size_t)data->animations_count, sizeof(float));
		for (cgltf_size ai = 0; ai < data->animations_count; ++ai) {
			const cgltf_animation *anim = &data->animations[ai];
			float tMin = 1e30f, tMax = -1e30f;
			for (cgltf_size si = 0; si < anim->samplers_count; ++si) {
				const cgltf_accessor *inAcc = anim->samplers[si].input;
				if (!inAcc || inAcc->count == 0) continue;
				float t0 = 0, t1 = 0;
				cgltf_accessor_read_float(inAcc, 0, &t0, 1);
				cgltf_accessor_read_float(inAcc, inAcc->count - 1, &t1, 1);
				if (t0 < tMin) tMin = t0;
				if (t1 > tMax) tMax = t1;
			}
			if (tMin > tMax) { tMin = 0.0f; tMax = 0.0f; }
			animTimeMin[ai] = tMin;
			animTimeMax[ai] = tMax;
			float duration = tMax - tMin;
			int N = 1 + (int)ceilf(duration * fps);
			if (N < 1) N = 1;
			animFrames[ai] = N;
			if (skeletalMode) totalAnimFrames += N;
		}
	}

	int numFrames = skeletalMode ? totalAnimFrames : morphFrames;
	if (numFrames < 1) numFrames = 1;
	for (int f = 0; f < numFrames; ++f) {
		mc_model_add_frame(out);
	}

	/* ---- Skeleton: copy bind TRS for every joint, default per-frame
	   pose to bind.  In skeletal mode the per-frame pose is overwritten
	   below from sampled animation channels. */
	if (totalJoints > 0) {
		mc_model_set_joints(out, totalJoints);
		for (cgltf_size si = 0; si < data->skins_count; ++si) {
			const cgltf_skin *sk = &data->skins[si];
			for (cgltf_size j = 0; j < sk->joints_count; ++j) {
				cgltf_size idx = (cgltf_size)(sk->joints[j] - data->nodes);
				int gj = gltfNode_to_joint[idx];
				if (gj < 0) continue;
				const cgltf_node *jn = sk->joints[j];
				mc_joint_t *mj = &out->joints[gj];
				mc_q_strncpy(mj->name, jn->name ? jn->name : "joint", sizeof(mj->name));
				int parent = -1;
				if (jn->parent) {
					cgltf_size pidx = (cgltf_size)(jn->parent - data->nodes);
					if (pidx < data->nodes_count) {
						int gp = gltfNode_to_joint[pidx];
						if (gp >= 0) parent = gp;
					}
				}
				mj->parent = parent;
				if (jn->has_translation) memcpy(mj->bindTrans, jn->translation, sizeof(mj->bindTrans));
				if (jn->has_rotation)    memcpy(mj->bindRot,   jn->rotation,    sizeof(mj->bindRot));
				else { mj->bindRot[0] = mj->bindRot[1] = mj->bindRot[2] = 0; mj->bindRot[3] = 1; }
				if (jn->has_scale) memcpy(mj->bindScale, jn->scale, sizeof(mj->bindScale));
				else { mj->bindScale[0] = mj->bindScale[1] = mj->bindScale[2] = 1; }
				for (int f = 0; f < out->numFrames; ++f) {
					mc_joint_pose_t *jp = &out->jointPoses[(size_t)f * out->numJoints + gj];
					memcpy(jp->trans, mj->bindTrans, sizeof(jp->trans));
					memcpy(jp->rot,   mj->bindRot,   sizeof(jp->rot));
					memcpy(jp->scale, mj->bindScale, sizeof(jp->scale));
				}
			}
		}
		MC_LOG("gltf: imported skeleton with %d joints%s\n", totalJoints,
			   skeletalMode ? " (skeletal animations detected)" : "");
	}

	/* ---- Sample per-joint TRS animation channels into per-frame poses
	   and append one mc_animation_t entry per source animation so the
	   IQM / MDR writers can re-emit the timeline losslessly. */
	if (skeletalMode) {
		float fps = fps_hint > 0.0f ? fps_hint : 15.0f;
		int frameCursor = 0;
		for (cgltf_size ai = 0; ai < data->animations_count; ++ai) {
			const cgltf_animation *anim = &data->animations[ai];
			int N = animFrames[ai];
			float tMin = animTimeMin[ai];

			mc_animation_t *am = mc_model_add_animation(out);
			mc_q_strncpy(am->name, anim->name ? anim->name : "anim", sizeof(am->name));
			am->firstFrame = frameCursor;
			am->numFrames = N;
			am->fps = fps;
			am->loopFrames = N;

			/* For every channel that targets a registered joint, evaluate
			   the sampler at each per-frame time and overwrite the matching
			   TRS slot of jointPoses. */
			for (cgltf_size ci = 0; ci < anim->channels_count; ++ci) {
				const cgltf_animation_channel *ch = &anim->channels[ci];
				if (!ch->target_node || !ch->sampler) continue;
				cgltf_size nidx = (cgltf_size)(ch->target_node - data->nodes);
				if (nidx >= data->nodes_count) continue;
				int gj = gltfNode_to_joint[nidx];
				if (gj < 0) continue;
				int comp = 3;
				if (ch->target_path == cgltf_animation_path_type_rotation) comp = 4;
				else if (ch->target_path != cgltf_animation_path_type_translation
						 && ch->target_path != cgltf_animation_path_type_scale)
					continue;
				for (int t = 0; t < N; ++t) {
					float time = tMin + (float)t / fps;
					float val[4];
					sampler_evaluate(ch->sampler, time, comp, val);
					mc_joint_pose_t *jp = &out->jointPoses[(size_t)(frameCursor + t) * out->numJoints + gj];
					if (ch->target_path == cgltf_animation_path_type_translation)
						memcpy(jp->trans, val, sizeof(float) * 3);
					else if (ch->target_path == cgltf_animation_path_type_rotation) {
						quat_normalize(val);
						memcpy(jp->rot, val, sizeof(float) * 4);
					} else
						memcpy(jp->scale, val, sizeof(float) * 3);
				}
			}
			frameCursor += N;
		}
		out->fps = fps;
	}

	/* For every visible mesh primitive, create a surface holding numFrames frames. */
	for (cgltf_size i = 0; i < data->nodes_count; ++i) {
		const cgltf_node *node = &data->nodes[i];
		if (!node->mesh)
			continue;
		float world[16];
		q3_compose_world(node, world);

		const cgltf_mesh *mesh = node->mesh;
		for (cgltf_size pi = 0; pi < mesh->primitives_count; ++pi) {
			const cgltf_primitive *prim = &mesh->primitives[pi];
			if (prim->type != cgltf_primitive_type_triangles) {
				MC_LOG("gltf: skipping non-triangle primitive on '%s'\n",
					   node->name ? node->name : (mesh->name ? mesh->name : "?"));
				continue;
			}

			const cgltf_accessor *posAcc = NULL, *normAcc = NULL, *uvAcc = NULL;
			const cgltf_accessor *jointsAcc = NULL, *weightsAcc = NULL;
			for (cgltf_size ai = 0; ai < prim->attributes_count; ++ai) {
				const cgltf_attribute *attr = &prim->attributes[ai];
				if (attr->type == cgltf_attribute_type_position)
					posAcc = attr->data;
				else if (attr->type == cgltf_attribute_type_normal)
					normAcc = attr->data;
				else if (attr->type == cgltf_attribute_type_texcoord && !uvAcc)
					uvAcc = attr->data;
				else if (attr->type == cgltf_attribute_type_joints && !jointsAcc)
					jointsAcc = attr->data;
				else if (attr->type == cgltf_attribute_type_weights && !weightsAcc)
					weightsAcc = attr->data;
			}
			if (!posAcc)
				continue;

			mc_surface_t *s = mc_model_add_surface(out);
			const char *baseName = node->name ? node->name : (mesh->name ? mesh->name : "surface");
			if (mesh->primitives_count > 1) {
				snprintf(s->name, sizeof(s->name), "%s_%u", baseName, (unsigned)pi);
			} else {
				mc_q_strncpy(s->name, baseName, sizeof(s->name));
			}

			/* Restore the part label from node extras. */
			if (node->extras.data && node->extras.data[0]) {
				char part[16];
				if (extras_get_string(node->extras.data, "q3_part", part, sizeof(part)))
					mc_q_strncpy(s->part, part, sizeof(s->part));
				int lodv = 0;
				if (extras_get_int(node->extras.data, "q3_lod", &lodv))
					s->lod = lodv;
			}
			/* Convention fallbacks when extras are missing. */
			if (!s->part[0])
				infer_part_from_node(node, s->part, sizeof(s->part));
			if (s->lod == 0)
				s->lod = infer_lod_from_node(node);

			resolve_material(prim->material, data, base_dir, gltf_dir, s);

			int numVerts = (int)posAcc->count;
			int numIdx = prim->indices ? (int)prim->indices->count : numVerts;
			int numTris = numIdx / 3;
			mc_surface_alloc(s, numVerts, numTris, numFrames);

			/* Frame-0 positions (transformed by node world matrix). */
			float *positions = (float *)mc_calloc((size_t)numVerts * 3, sizeof(float));
			read_floats(posAcc, positions, 3);
			for (int v = 0; v < numVerts; ++v) {
				mat4_transform_point(world, &positions[v * 3], &s->xyz[v * 3]);
			}

			/* Frame-0 normals (or default +Z). */
			if (normAcc) {
				float *normals = (float *)mc_calloc((size_t)numVerts * 3, sizeof(float));
				read_floats(normAcc, normals, 3);
				for (int v = 0; v < numVerts; ++v) {
					mat4_transform_dir(world, &normals[v * 3], &s->normal[v * 3]);
					normalize3(&s->normal[v * 3]);
				}
				free(normals);
			} else {
				for (int v = 0; v < numVerts; ++v) {
					s->normal[v * 3 + 2] = 1.0f;
				}
			}

			/* Morph targets: deltas relative to frame 0 in glTF, baked into
			   absolute positions per frame for the in-memory model.  We
			   transform the deltas through the rotation part of the world
			   matrix so animations stay correct after the +Y-up -> +Z-up
			   root rotation our writer applies. */
			for (int t = 0; t < (int)prim->targets_count && (t + 1) < numFrames; ++t) {
				const cgltf_morph_target *mt = &prim->targets[t];
				const cgltf_accessor *tposAcc = NULL;
				const cgltf_accessor *tnormAcc = NULL;
				for (cgltf_size ai = 0; ai < mt->attributes_count; ++ai) {
					if (mt->attributes[ai].type == cgltf_attribute_type_position)
						tposAcc = mt->attributes[ai].data;
					else if (mt->attributes[ai].type == cgltf_attribute_type_normal)
						tnormAcc = mt->attributes[ai].data;
				}
				if (!tposAcc)
					continue;
				float *deltas = (float *)mc_calloc((size_t)numVerts * 3, sizeof(float));
				read_floats(tposAcc, deltas, 3);
				for (int v = 0; v < numVerts; ++v) {
					float d[3] = {deltas[v * 3 + 0], deltas[v * 3 + 1], deltas[v * 3 + 2]};
					float dw[3];
					mat4_transform_dir(world, d, dw);
					float *out_xyz = s->xyz + (size_t)(t + 1) * numVerts * 3 + v * 3;
					out_xyz[0] = s->xyz[v * 3 + 0] + dw[0];
					out_xyz[1] = s->xyz[v * 3 + 1] + dw[1];
					out_xyz[2] = s->xyz[v * 3 + 2] + dw[2];
				}
				free(deltas);
				if (tnormAcc) {
					float *ndelta = (float *)mc_calloc((size_t)numVerts * 3, sizeof(float));
					read_floats(tnormAcc, ndelta, 3);
					for (int v = 0; v < numVerts; ++v) {
						float d[3] = {ndelta[v * 3 + 0], ndelta[v * 3 + 1], ndelta[v * 3 + 2]};
						float dw[3];
						mat4_transform_dir(world, d, dw);
						float *on = s->normal + (size_t)(t + 1) * numVerts * 3 + v * 3;
						on[0] = s->normal[v * 3 + 0] + dw[0];
						on[1] = s->normal[v * 3 + 1] + dw[1];
						on[2] = s->normal[v * 3 + 2] + dw[2];
						normalize3(on);
					}
					free(ndelta);
				} else {
					/* Reuse base normals when no morph normal data is provided. */
					for (int v = 0; v < numVerts; ++v) {
						float *on = s->normal + (size_t)(t + 1) * numVerts * 3 + v * 3;
						memcpy(on, s->normal + v * 3, sizeof(float) * 3);
					}
				}
			}
			/* Frames beyond targets_count clamp to the last available pose. */
			for (int f = (int)prim->targets_count + 1; f < numFrames; ++f) {
				int src = (int)prim->targets_count;
				memcpy(s->xyz + (size_t)f * numVerts * 3, s->xyz + (size_t)src * numVerts * 3,
					   (size_t)numVerts * 3 * sizeof(float));
				memcpy(s->normal + (size_t)f * numVerts * 3, s->normal + (size_t)src * numVerts * 3,
					   (size_t)numVerts * 3 * sizeof(float));
			}

			free(positions);

			/* UVs. */
			if (uvAcc) {
				read_floats(uvAcc, s->st, 2);
			}

			/* Indices.  glTF uses CCW front-facing while Q3 (MD3/MDR/IQM)
			   uses CW from outside.  Swap v1 and v2 of each triangle so the
			   intermediate model keeps the engine's expected CW convention
			   for downstream MD3/MDR/IQM writers. */
			if (prim->indices) {
				int *tmp = (int *)mc_calloc((size_t)numIdx, sizeof(int));
				read_indices(prim->indices, tmp);
				for (int t = 0; t < numTris; ++t) {
					s->indices[t * 3 + 0] = tmp[t * 3 + 0];
					s->indices[t * 3 + 1] = tmp[t * 3 + 2];
					s->indices[t * 3 + 2] = tmp[t * 3 + 1];
				}
				free(tmp);
			} else {
				for (int t = 0; t < numTris; ++t) {
					s->indices[t * 3 + 0] = t * 3 + 0;
					s->indices[t * 3 + 1] = t * 3 + 2;
					s->indices[t * 3 + 2] = t * 3 + 1;
				}
			}

			/* JOINTS_0 / WEIGHTS_0: store up to 4 influences per vertex.
			   Local skin joint indices are remapped to absolute m->joints
			   indices via gltfNode_to_joint[]. */
			if (jointsAcc && weightsAcc && jointsAcc->count == (cgltf_size)numVerts &&
				weightsAcc->count == (cgltf_size)numVerts) {
				mc_surface_alloc_blend(s);
				const cgltf_skin *sk = node->skin;
				float *jf = (float *)mc_calloc((size_t)numVerts * 4, sizeof(float));
				float *wf = (float *)mc_calloc((size_t)numVerts * 4, sizeof(float));
				read_floats(jointsAcc, jf, 4);
				read_floats(weightsAcc, wf, 4);
				for (int v = 0; v < numVerts; ++v) {
					float sum = 0;
					for (int k = 0; k < 4; ++k) {
						int local = (int)(jf[v * 4 + k] + 0.5f);
						int global = 0;
						if (sk && local >= 0 && (cgltf_size)local < sk->joints_count && gltfNode_to_joint) {
							cgltf_size nidx = (cgltf_size)(sk->joints[local] - data->nodes);
							if (nidx < data->nodes_count && gltfNode_to_joint[nidx] >= 0)
								global = gltfNode_to_joint[nidx];
						}
						if (global > 255) global = 255;
						s->blendIndices[v * 4 + k] = (unsigned char)global;
						float w = wf[v * 4 + k];
						if (w < 0) w = 0;
						s->blendWeights[v * 4 + k] = w;
						sum += w;
					}
					if (sum > 1e-6f)
						for (int k = 0; k < 4; ++k) s->blendWeights[v * 4 + k] /= sum;
				}
				free(jf);
				free(wf);
			}
		}
	}

	/* Tags. */
	for (cgltf_size i = 0; i < data->nodes_count; ++i) {
		const cgltf_node *node = &data->nodes[i];
		if (!is_tag_node(node))
			continue;
		mc_tag_t *tag = mc_model_add_tag_slot(out, out->numFrames);
		/* MD3 tag names include the "tag_" prefix verbatim; preserve
		   the full name so a glTF -> MD3 round-trip is byte-stable. */
		mc_q_strncpy(tag->name, node->name, sizeof(tag->name));
		if (node->extras.data && node->extras.data[0]) {
			char part[16];
			if (extras_get_string(node->extras.data, "q3_part", part, sizeof(part)))
				mc_q_strncpy(tag->part, part, sizeof(tag->part));
			int lodv = 0;
			if (extras_get_int(node->extras.data, "q3_lod", &lodv))
				tag->lod = lodv;
		}
		if (!tag->part[0]) {
			/* Tags inherit the part of their owning subtree. */
			infer_part_from_node(node, tag->part, sizeof(tag->part));
		}
		if (tag->lod == 0)
			tag->lod = infer_lod_from_node(node);

		float world[16];
		q3_compose_world(node, world);

		float fwd[3] = {1, 0, 0};
		float left[3] = {0, 1, 0};
		float up[3] = {0, 0, 1};
		float a0[3], a1[3], a2[3];
		mat4_transform_dir(world, fwd, a0);
		mat4_transform_dir(world, left, a1);
		mat4_transform_dir(world, up, a2);
		normalize3(a0);
		normalize3(a1);
		normalize3(a2);
		/* Broadcast the bind-pose tag transform across every output frame -
		   tags are static in glTF since we do not currently sample
		   skeletal animation tracks for them. */
		int tagIdx = out->numTags - 1;
		for (int f = 0; f < out->numFrames; ++f) {
			mc_tag_t *t = &out->tags[(size_t)f * out->numTags + tagIdx];
			mc_q_strncpy(t->name, tag->name, sizeof(t->name));
			mc_q_strncpy(t->part, tag->part, sizeof(t->part));
			t->lod = tag->lod;
			memcpy(t->axis[0], a0, sizeof(a0));
			memcpy(t->axis[1], a1, sizeof(a1));
			memcpy(t->axis[2], a2, sizeof(a2));
			t->origin[0] = world[12];
			t->origin[1] = world[13];
			t->origin[2] = world[14];
		}
	}

	/* ---- Per-frame tag animation ----
	   The writer emits translation+rotation channels for every tag node
	   in every animation so MD3 -> glTF -> MD3 preserves moving weapon
	   attachment points etc.  Sample those channels and overwrite the
	   broadcast frames above.  We only need to override the tag node's
	   own local TRS - the parent chain's per-frame state is irrelevant
	   because pivots are skipped (q3_compose_world ignores them) and
	   the remaining ancestors carry no per-frame animation. */
	if (data->animations_count > 0 && out->numTags > 0 && out->numFrames > 0) {
		/* Build node -> tag-index map. */
		int *nodeToTag = (int *)mc_malloc(sizeof(int) * data->nodes_count);
		for (cgltf_size i = 0; i < data->nodes_count; ++i) nodeToTag[i] = -1;
		int tagCursor = 0;
		for (cgltf_size i = 0; i < data->nodes_count; ++i) {
			if (!is_tag_node(&data->nodes[i])) continue;
			if (tagCursor < out->numTags) nodeToTag[i] = tagCursor;
			++tagCursor;
		}
		/* Cache parent-world (without the tag's own local transform) per
		   tag so we don't recompute it for every frame. */
		float (*parentWorld)[16] = (float(*)[16])mc_malloc(sizeof(float[16]) * out->numTags);
		for (int ti = 0; ti < out->numTags; ++ti) mat4_identity(parentWorld[ti]);
		for (cgltf_size i = 0; i < data->nodes_count; ++i) {
			int ti = nodeToTag[i];
			if (ti < 0) continue;
			const cgltf_node *node = &data->nodes[i];
			if (node->parent)
				q3_compose_world(node->parent, parentWorld[ti]);
			else
				mat4_identity(parentWorld[ti]);
		}

		float fps = fps_hint > 0.0f ? fps_hint : 15.0f;
		int frameCursor = 0;
		for (cgltf_size ai = 0; ai < data->animations_count; ++ai) {
			const cgltf_animation *anim = &data->animations[ai];
			int N = animFrames ? animFrames[ai] : 0;
			if (N <= 0) {
				/* Non-skeletal mode: derive N from the longest sampler. */
				N = out->numFrames;
			}
			float tMin = animTimeMin ? animTimeMin[ai] : 0.0f;
			int firstFrame = skeletalMode ? frameCursor : 0;
			/* Prefer the writer-stamped q3_first_frame extra so morph
			   mode can land sampled tags at the right global frame. */
			if (!skeletalMode && anim->extras.data && anim->extras.data[0]) {
				int ff = 0;
				if (extras_get_int(anim->extras.data, "q3_first_frame", &ff))
					firstFrame = ff;
			}

			/* Per-tag accumulated TRS overrides (start from bind defaults). */
			for (cgltf_size ci = 0; ci < anim->channels_count; ++ci) {
				const cgltf_animation_channel *ch = &anim->channels[ci];
				if (!ch->target_node || !ch->sampler) continue;
				cgltf_size nidx = (cgltf_size)(ch->target_node - data->nodes);
				if (nidx >= data->nodes_count) continue;
				int ti = nodeToTag[nidx];
				if (ti < 0) continue;
				int isTrans = (ch->target_path == cgltf_animation_path_type_translation);
				int isRot = (ch->target_path == cgltf_animation_path_type_rotation);
				if (!isTrans && !isRot) continue;

				const cgltf_node *node = &data->nodes[nidx];
				/* Pull bind TRS for the components we don't sample. */
				float bindT[3] = {0,0,0}, bindR[4] = {0,0,0,1}, bindS[3] = {1,1,1};
				if (node->has_translation) memcpy(bindT, node->translation, sizeof(bindT));
				if (node->has_rotation)    memcpy(bindR, node->rotation, sizeof(bindR));
				if (node->has_scale)       memcpy(bindS, node->scale, sizeof(bindS));

				for (int f = 0; f < N; ++f) {
					int outFrame = firstFrame + f;
					if (outFrame >= out->numFrames) break;
					float time = tMin + (float)f / fps;
					float val[4];
					sampler_evaluate(ch->sampler, time, isRot ? 4 : 3, val);
					float trans[3], rot[4], scale[3];
					memcpy(trans, bindT, sizeof(trans));
					memcpy(rot,   bindR, sizeof(rot));
					memcpy(scale, bindS, sizeof(scale));
					if (isTrans) memcpy(trans, val, sizeof(trans));
					else { quat_normalize(val); memcpy(rot, val, sizeof(rot)); }

					/* Build local mat4 (column-major) from TRS. */
					float qx=rot[0], qy=rot[1], qz=rot[2], qw=rot[3];
					float xx=qx*qx, yy=qy*qy, zz=qz*qz;
					float xy=qx*qy, xz=qx*qz, yz=qy*qz;
					float wx=qw*qx, wy=qw*qy, wz=qw*qz;
					float local[16] = {
						(1-2*(yy+zz))*scale[0], (2*(xy+wz))*scale[0],   (2*(xz-wy))*scale[0],   0,
						(2*(xy-wz))*scale[1],   (1-2*(xx+zz))*scale[1], (2*(yz+wx))*scale[1],   0,
						(2*(xz+wy))*scale[2],   (2*(yz-wx))*scale[2],   (1-2*(xx+yy))*scale[2], 0,
						trans[0], trans[1], trans[2], 1,
					};
					float world2[16];
					mat4_mul(parentWorld[ti], local, world2);
					float fwd[3] = {1,0,0}, left[3] = {0,1,0}, up[3] = {0,0,1};
					float a0[3], a1[3], a2[3];
					mat4_transform_dir(world2, fwd, a0);
					mat4_transform_dir(world2, left, a1);
					mat4_transform_dir(world2, up, a2);
					normalize3(a0); normalize3(a1); normalize3(a2);
					mc_tag_t *tt = &out->tags[(size_t)outFrame * out->numTags + ti];
					/* Only override the channel we sampled - other channel
					   merges later in this loop or relies on this being
					   the only channel for this tag (writer always emits
					   both translation+rotation, so the second pass for
					   the rotation channel will overwrite axes correctly,
					   and the translation pass overwrites origin).  This
					   works because building local from (trans,rot) here
					   uses bind values for the un-sampled component. */
					if (isTrans) {
						tt->origin[0] = world2[12];
						tt->origin[1] = world2[13];
						tt->origin[2] = world2[14];
					} else {
						memcpy(tt->axis[0], a0, sizeof(a0));
						memcpy(tt->axis[1], a1, sizeof(a1));
						memcpy(tt->axis[2], a2, sizeof(a2));
					}
				}
			}
			if (skeletalMode) frameCursor += N;
		}
		free(parentWorld);
		free(nodeToTag);
	}

	/* Convention-based player-bundle inference: if surfaces collectively
	   cover head + upper + lower we treat the file as a player even when
	   the q3_player asset extra is missing. */
	if (!out->is_player_bundle) {
		int seen_head = 0, seen_upper = 0, seen_lower = 0;
		for (int i = 0; i < out->numSurfaces; ++i) {
			if (!strcmp(out->surfaces[i].part, "head")) seen_head = 1;
			else if (!strcmp(out->surfaces[i].part, "upper")) seen_upper = 1;
			else if (!strcmp(out->surfaces[i].part, "lower")) seen_lower = 1;
		}
		if (seen_head && seen_upper && seen_lower) {
			out->is_player_bundle = 1;
			MC_LOG("gltf: inferred player-bundle layout from head/upper/lower naming\n");
		}
	}

	/* ---- Skeletal re-bake.  When animations sampled per-frame joint
	   poses, recompute every skinned surface's per-frame xyz/normal so
	   the in-memory model matches what the IQM / MDR writers expect:
	   frame 0 holds bind-pose vertices, frames 1..numFrames-1 hold the
	   linearly skinned positions for that frame's joint TRS. */
	if (skeletalMode && out->numJoints > 0) {
		int J = out->numJoints;
		float (*bindAbs)[3][4] = (float (*)[3][4])mc_calloc((size_t)J, sizeof(*bindAbs));
		float (*bindAbsInv)[3][4] = (float (*)[3][4])mc_calloc((size_t)J, sizeof(*bindAbsInv));
		for (int j = 0; j < J; ++j) {
			const mc_joint_t *mj = &out->joints[j];
			float local[3][4];
			mat34_from_trs(mj->bindTrans, mj->bindRot, mj->bindScale, local);
			if (mj->parent >= 0)
				mat34_mul(bindAbs[mj->parent], local, bindAbs[j]);
			else
				memcpy(bindAbs[j], local, sizeof(local));
			mat34_invert(bindAbs[j], bindAbsInv[j]);
		}

		float (*absMats)[3][4] = (float (*)[3][4])mc_calloc((size_t)J, sizeof(*absMats));
		float (*skinMats)[3][4] = (float (*)[3][4])mc_calloc((size_t)J, sizeof(*skinMats));

		for (int si = 0; si < out->numSurfaces; ++si) {
			mc_surface_t *s = &out->surfaces[si];
			if (!s->blendIndices || !s->blendWeights) continue;

			/* Snapshot bind positions / normals from frame 0 before we
			   start overwriting other frames. */
			float *bindPos = (float *)mc_malloc(sizeof(float) * (size_t)s->numVerts * 3);
			float *bindNrm = (float *)mc_malloc(sizeof(float) * (size_t)s->numVerts * 3);
			memcpy(bindPos, s->xyz, sizeof(float) * (size_t)s->numVerts * 3);
			memcpy(bindNrm, s->normal, sizeof(float) * (size_t)s->numVerts * 3);

			for (int f = 1; f < out->numFrames; ++f) {
				/* Build absolute joint matrices for this frame. */
				for (int j = 0; j < J; ++j) {
					const mc_joint_pose_t *jp = &out->jointPoses[(size_t)f * J + j];
					float local[3][4];
					mat34_from_trs(jp->trans, jp->rot, jp->scale, local);
					int parent = out->joints[j].parent;
					if (parent >= 0)
						mat34_mul(absMats[parent], local, absMats[j]);
					else
						memcpy(absMats[j], local, sizeof(local));
					mat34_mul(absMats[j], bindAbsInv[j], skinMats[j]);
				}
				float *xyzF = s->xyz + (size_t)f * s->numVerts * 3;
				float *nrmF = s->normal + (size_t)f * s->numVerts * 3;
				for (int v = 0; v < s->numVerts; ++v) {
					float pos[3] = {0, 0, 0};
					float nrm[3] = {0, 0, 0};
					float wsum = 0.0f;
					for (int k = 0; k < 4; ++k) {
						float w = s->blendWeights[v * 4 + k];
						if (w <= 0) continue;
						unsigned int j = s->blendIndices[v * 4 + k];
						if ((int)j >= J) continue;
						float p[3];
						mat34_xform_point(skinMats[j], &bindPos[v * 3], p);
						pos[0] += w * p[0]; pos[1] += w * p[1]; pos[2] += w * p[2];
						float n[3];
						mat34_xform_dir(skinMats[j], &bindNrm[v * 3], n);
						nrm[0] += w * n[0]; nrm[1] += w * n[1]; nrm[2] += w * n[2];
						wsum += w;
					}
					if (wsum <= 1e-6f) {
						pos[0] = bindPos[v * 3 + 0];
						pos[1] = bindPos[v * 3 + 1];
						pos[2] = bindPos[v * 3 + 2];
						nrm[0] = bindNrm[v * 3 + 0];
						nrm[1] = bindNrm[v * 3 + 1];
						nrm[2] = bindNrm[v * 3 + 2];
					}
					xyzF[v * 3 + 0] = pos[0];
					xyzF[v * 3 + 1] = pos[1];
					xyzF[v * 3 + 2] = pos[2];
					normalize3(nrm);
					nrmF[v * 3 + 0] = nrm[0];
					nrmF[v * 3 + 1] = nrm[1];
					nrmF[v * 3 + 2] = nrm[2];
				}
			}
			free(bindPos);
			free(bindNrm);
		}
		free(bindAbs);
		free(bindAbsInv);
		free(absMats);
		free(skinMats);
		MC_LOG("gltf: sampled %d animation(s), %d joint frames\n",
			   (int)data->animations_count, totalAnimFrames);
	}
	free(animFrames);
	free(animTimeMin);
	free(animTimeMax);

	/* Compute frame bounds. */
	for (int f = 0; f < out->numFrames; ++f) {
		mc_frame_t *fr = &out->frames[f];
		float mins[3] = {1e30f, 1e30f, 1e30f};
		float maxs[3] = {-1e30f, -1e30f, -1e30f};
		for (int si = 0; si < out->numSurfaces; ++si) {
			const mc_surface_t *s = &out->surfaces[si];
			const float *xyz = s->xyz + (size_t)f * s->numVerts * 3;
			for (int v = 0; v < s->numVerts; ++v) {
				for (int k = 0; k < 3; ++k) {
					if (xyz[v * 3 + k] < mins[k])
						mins[k] = xyz[v * 3 + k];
					if (xyz[v * 3 + k] > maxs[k])
						maxs[k] = xyz[v * 3 + k];
				}
			}
		}
		if (mins[0] > maxs[0]) {
			mins[0] = mins[1] = mins[2] = 0;
			maxs[0] = maxs[1] = maxs[2] = 0;
		}
		memcpy(fr->bounds[0], mins, sizeof(mins));
		memcpy(fr->bounds[1], maxs, sizeof(maxs));
		fr->localOrigin[0] = fr->localOrigin[1] = fr->localOrigin[2] = 0;
		float cx = 0.5f * (mins[0] + maxs[0]);
		float cy = 0.5f * (mins[1] + maxs[1]);
		float cz = 0.5f * (mins[2] + maxs[2]);
		float r2 = 0;
		for (int si = 0; si < out->numSurfaces; ++si) {
			const mc_surface_t *s = &out->surfaces[si];
			const float *xyz = s->xyz + (size_t)f * s->numVerts * 3;
			for (int v = 0; v < s->numVerts; ++v) {
				float dx = xyz[v * 3 + 0] - cx;
				float dy = xyz[v * 3 + 1] - cy;
				float dz = xyz[v * 3 + 2] - cz;
				float d2 = dx * dx + dy * dy + dz * dz;
				if (d2 > r2)
					r2 = d2;
			}
		}
		fr->radius = sqrtf(r2);
		snprintf(fr->name, sizeof(fr->name), "frame%d", f);
	}

	cgltf_free(data);
	free(gltfNode_to_joint);
	return 0;
}
