/*
===========================================================================
modelconverter - glTF/GLB writer

Produces a glTF 2.0 document from the in-memory mc_model_t.  Multi-frame
MD3 / IQM data is emitted as a single base mesh per surface plus one
POSITION morph target per additional frame.  Tags are emitted as scene
graph nodes (extras = { "q3_tag": true }) so they round-trip cleanly when
the file is later re-imported by the same tool.
===========================================================================
*/

#define CGLTF_WRITE_IMPLEMENTATION
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-Wmissing-prototypes"
#endif
#include "cgltf_write.h"
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include "mc_common.h"

#include <ctype.h>
#include <float.h>
#include <math.h>

/* ------------------------------------------------------------------ */
/* Small dynamic byte buffer used to assemble the .bin payload        */
/* ------------------------------------------------------------------ */

typedef struct {
	unsigned char *data;
	size_t size;
	size_t cap;
} bin_buf_t;

static void bin_reserve(bin_buf_t *b, size_t extra) {
	if (b->size + extra <= b->cap)
		return;
	size_t cap = b->cap ? b->cap : 4096;
	while (cap < b->size + extra)
		cap *= 2;
	b->data = (unsigned char *)mc_realloc(b->data, cap);
	b->cap = cap;
}

static size_t bin_append(bin_buf_t *b, const void *src, size_t n) {
	bin_reserve(b, n + 4);
	size_t ofs = b->size;
	memcpy(b->data + ofs, src, n);
	b->size += n;
	/* glTF 2.0 requires buffer view offsets to be aligned to the size of
	   their component type. Pad to 4 for safety. */
	while (b->size & 3u) {
		b->data[b->size++] = 0;
	}
	return ofs;
}

/* ------------------------------------------------------------------ */
/* Helper to produce a simple lowercase shader-friendly node name.    */
/* ------------------------------------------------------------------ */

static char *dup_str(const char *s) {
	if (!s)
		return NULL;
	size_t n = strlen(s) + 1;
	char *r = (char *)mc_malloc(n);
	memcpy(r, s, n);
	return r;
}

/* ------------------------------------------------------------------ */
/* Texture file -> mime type                                          */
/* ------------------------------------------------------------------ */

static char *guess_mime(const char *path) {
	const char *dot = strrchr(path, '.');
	if (!dot)
		return dup_str("image/png");
	if (!strcasecmp(dot, ".jpg") || !strcasecmp(dot, ".jpeg"))
		return dup_str("image/jpeg");
	if (!strcasecmp(dot, ".png"))
		return dup_str("image/png");
	if (!strcasecmp(dot, ".tga"))
		return dup_str("image/tga");
	if (!strcasecmp(dot, ".webp"))
		return dup_str("image/webp");
	return dup_str("image/png");
}

/* ------------------------------------------------------------------ */
/* Allocator helpers                                                  */
/* ------------------------------------------------------------------ */

#define ALLOC_ARR(T, n) ((T *)mc_calloc((n) > 0 ? (n) : 1, sizeof(T)))

/* Convert a Q3 tag axis (3 row-vectors: forward, left, up) into a unit
   quaternion stored as (x, y, z, w) - the order glTF uses. */
static void q3_axis_to_quat(const float axis[3][3], float out[4]) {
	/* Standard column-major rotation matrix from axis (axis[col][row]). */
	float m00 = axis[0][0], m10 = axis[0][1], m20 = axis[0][2];
	float m01 = axis[1][0], m11 = axis[1][1], m21 = axis[1][2];
	float m02 = axis[2][0], m12 = axis[2][1], m22 = axis[2][2];
	float trace = m00 + m11 + m22;
	float qx, qy, qz, qw;
	if (trace > 0.0f) {
		float s = 0.5f / sqrtf(trace + 1.0f);
		qw = 0.25f / s;
		qx = (m21 - m12) * s;
		qy = (m02 - m20) * s;
		qz = (m10 - m01) * s;
	} else if (m00 > m11 && m00 > m22) {
		float s = 2.0f * sqrtf(1.0f + m00 - m11 - m22);
		qw = (m21 - m12) / s;
		qx = 0.25f * s;
		qy = (m01 + m10) / s;
		qz = (m02 + m20) / s;
	} else if (m11 > m22) {
		float s = 2.0f * sqrtf(1.0f + m11 - m00 - m22);
		qw = (m02 - m20) / s;
		qx = (m01 + m10) / s;
		qy = 0.25f * s;
		qz = (m12 + m21) / s;
	} else {
		float s = 2.0f * sqrtf(1.0f + m22 - m00 - m11);
		qw = (m10 - m01) / s;
		qx = (m02 + m20) / s;
		qy = (m12 + m21) / s;
		qz = 0.25f * s;
	}
	float n = sqrtf(qx * qx + qy * qy + qz * qz + qw * qw);
	if (n > 0.0f) {
		qx /= n;
		qy /= n;
		qz /= n;
		qw /= n;
	}
	out[0] = qx;
	out[1] = qy;
	out[2] = qz;
	out[3] = qw;
}

/* Resolve a virtual Q3 texture path (e.g. "models/foo/bar.tga") to an
   on-disk file by searching `roots` in order.  Tries the path as-is
   first, then substitutes the file extension with the most common image
   extensions.  Writes the resolved absolute path into `out_resolved`
   and the matching MIME type into `out_mime` (heap-allocated).  Returns
   1 if the file was found, 0 otherwise. */
static int resolve_texture_file(const char *texPath, const char *const *roots, char *out_resolved, size_t out_size,
								char **out_mime) {
	if (!texPath || !texPath[0])
		return 0;

	char stem[MC_MAX_PATH];
	mc_q_strncpy(stem, texPath, sizeof(stem));
	mc_strip_extension(stem);

	const char *orig_ext = strrchr(texPath, '.');
	static const char *fallback_exts[] = {".png", ".tga", ".jpg", ".jpeg", ".webp", NULL};

	for (int rr = 0; !roots || roots[rr]; ++rr) {
		const char *root = roots ? roots[rr] : "";
		if (!root)
			break;
		for (int ei = -1; ei < 0 || fallback_exts[ei]; ++ei) {
			const char *ext = (ei < 0) ? (orig_ext ? orig_ext : "") : fallback_exts[ei];
			if (ei < 0 && !ext[0])
				continue;
			char rel[MC_MAX_PATH];
			snprintf(rel, sizeof(rel), "%s%s", stem, ext);
			char abs[MC_MAX_PATH];
			if (root[0])
				mc_join_path(abs, sizeof(abs), root, rel);
			else
				mc_q_strncpy(abs, rel, sizeof(abs));
			FILE *f = fopen(abs, "rb");
			if (f) {
				fclose(f);
				mc_q_strncpy(out_resolved, abs, out_size);
				if (out_mime)
					*out_mime = guess_mime(abs);
				return 1;
			}
		}
		if (!roots)
			break;
	}
	return 0;
}

/* Push one image+texture entry referencing `texPath`.  When asset_roots
   is non-NULL and the file is found on disk, the bytes are either
   embedded into the .glb binary chunk (as_glb=1) or copied next to the
   output (as_glb=0).  Otherwise the path is written to the URI verbatim
   so the engine VFS can still resolve it.  Returns the new cgltf_texture
   pointer, or NULL when no entry was added. */
static cgltf_texture *push_texture(cgltf_data *data, cgltf_image *images, cgltf_texture *textures,
								   cgltf_sampler *samplers, const char *texPath, const char *surfName,
								   const char *const *asset_roots, int as_glb, const char *out_dir, bin_buf_t *bin) {
	if (!textures || !texPath || !texPath[0])
		return NULL;
	char resolved[MC_MAX_PATH];
	resolved[0] = 0;
	char *mime = NULL;
	int found = resolve_texture_file(texPath, asset_roots, resolved, sizeof(resolved), &mime);

	/* If we couldn't find the texture on disk, embed a tiny magenta/black
	   checker placeholder so the material is still visibly broken in any
	   viewer (much easier to spot than a silent baseColorFactor fallback)
	   and so the asset stays self-contained. The original q3 path is
	   preserved in the q3_shader extras so a round-trip back to MD3/MDR
	   keeps the reference intact. */
	if (!found) {
		if (mime)
			free(mime);
		MC_LOG("warn: texture not found on disk, embedding magenta/black placeholder for: %s\n", texPath);
		size_t pngSz = 0;
		unsigned char *png = mc_image_make_missing_placeholder(&pngSz);
		if (!png)
			return NULL;
		cgltf_image *img = &images[data->images_count];
		if (as_glb) {
			size_t ofs = bin_append(bin, png, pngSz);
			cgltf_buffer_view *bv = &data->buffer_views[data->buffer_views_count++];
			bv->buffer = &data->buffers[0];
			bv->offset = ofs;
			bv->size = pngSz;
			img->buffer_view = bv;
			img->mime_type = dup_str("image/png");
			img->name = dup_str(mc_basename(texPath));
			free(png);
		} else {
			char base[MC_MAX_PATH];
			mc_q_strncpy(base, mc_basename(texPath), sizeof(base));
			char *dot = strrchr(base, '.');
			if (dot) *dot = 0;
			size_t bl = strlen(base);
			if (bl + 5 < sizeof(base))
				memcpy(base + bl, ".png", 5);
			if (out_dir && out_dir[0]) {
				char dst[MC_MAX_PATH];
				mc_join_path(dst, sizeof(dst), out_dir, base);
				mc_write_file(dst, png, pngSz);
			}
			img->uri = dup_str(base);
			img->mime_type = dup_str("image/png");
			img->name = dup_str(mc_basename(texPath));
			free(png);
		}
		cgltf_texture *tx = &textures[data->textures_count];
		tx->image = img;
		tx->sampler = &samplers[0];
		tx->name = dup_str(surfName);
		data->images_count++;
		data->textures_count++;
		return tx;
	}

	cgltf_image *img = &images[data->images_count];

	/* Decide if we need to transcode to PNG.  glTF 2.0 only allows PNG and
	   JPEG image media types; TGA / BMP / etc. embedded in the binary chunk
	   would be silently dropped by Blender and most online viewers.  We
	   keep PNG/JPEG as-is and convert anything else through stb_image. */
	const char *mime_str = mime ? mime : guess_mime(resolved);
	int needs_transcode = !(mime_str
		&& (!strcmp(mime_str, "image/png") || !strcmp(mime_str, "image/jpeg")));

	if (as_glb) {
		/* Embed via buffer view appended to the shared bin payload. */
		size_t sz = 0;
		unsigned char *bytes = mc_read_file(resolved, &sz);
		if (bytes && sz > 0) {
			unsigned char *encoded = bytes;
			size_t encoded_sz = sz;
			const char *out_mime = mime_str;
			if (needs_transcode) {
				size_t pngSz = 0;
				unsigned char *png = mc_image_to_png(bytes, sz, &pngSz);
				if (png) {
					encoded = png;
					encoded_sz = pngSz;
					out_mime = "image/png";
				} else {
					MC_ERR("texture: failed to transcode '%s' to PNG; refusing to embed non-compliant raw bytes (glTF 2.0 only allows PNG/JPEG)\n", resolved);
					free(bytes);
					if (mime) free(mime);
					return NULL;
				}
			}
			size_t ofs = bin_append(bin, encoded, encoded_sz);
			cgltf_buffer_view *bv = &data->buffer_views[data->buffer_views_count++];
			bv->buffer = &data->buffers[0];
			bv->offset = ofs;
			bv->size = encoded_sz;
			img->buffer_view = bv;
			img->mime_type = dup_str(out_mime);
			img->name = dup_str(mc_basename(texPath));
			if (encoded != bytes)
				free(encoded);
			free(bytes);
			if (mime)
				free(mime);
		} else {
			if (mime)
				free(mime);
			return NULL;
		}
	} else {
		/* Copy file next to .gltf and reference by basename.  Transcode
		   non-PNG/JPEG sources to PNG so the resulting .gltf works in
		   any viewer. */
		char base[MC_MAX_PATH];
		mc_q_strncpy(base, mc_basename(resolved), sizeof(base));
		const char *out_mime = mime_str;
		if (needs_transcode) {
			/* Replace extension with .png in the basename. */
			char *dot = strrchr(base, '.');
			if (dot)
				*dot = 0;
			size_t bl = strlen(base);
			if (bl + 5 < sizeof(base))
				memcpy(base + bl, ".png", 5);
			out_mime = "image/png";
		}
		if (out_dir && out_dir[0]) {
			char dst[MC_MAX_PATH];
			mc_join_path(dst, sizeof(dst), out_dir, base);
			size_t sz = 0;
			unsigned char *bytes = mc_read_file(resolved, &sz);
			if (bytes) {
				if (needs_transcode) {
					size_t pngSz = 0;
					unsigned char *png = mc_image_to_png(bytes, sz, &pngSz);
					if (png) {
						mc_write_file(dst, png, pngSz);
						free(png);
					} else {
						MC_ERR("texture: failed to transcode '%s' to PNG; refusing to copy non-compliant raw bytes\n", resolved);
						free(bytes);
						if (mime) free(mime);
						return NULL;
					}
				} else {
					mc_write_file(dst, bytes, sz);
				}
				free(bytes);
			}
		}
		img->uri = dup_str(base);
		img->mime_type = dup_str(out_mime);
		img->name = dup_str(mc_basename(texPath));
		if (mime)
			free(mime);
	}

	cgltf_texture *tx = &textures[data->textures_count];
	tx->image = img;
	tx->sampler = &samplers[0];
	tx->name = dup_str(surfName);
	data->images_count++;
	data->textures_count++;
	return tx;
}

/* ------------------------------------------------------------------ */
/* Main writer                                                        */
/* ------------------------------------------------------------------ */

int mc_save_gltf(const char *path, const mc_model_t *m, int as_glb, const char *const *asset_roots) {
	char out_dir[MC_MAX_PATH];
	mc_dirname(path, out_dir, sizeof(out_dir));

	int numFrames = m->numFrames > 0 ? m->numFrames : 1;
	int totalSurfs = m->numSurfaces;
	int totalTags = m->numTags;
	int hasSkin = (m->numJoints > 0);
	/* Count surfaces that carry blend data; only those get JOINTS_0 / WEIGHTS_0. */
	int skinnedSurfs = 0;
	if (hasSkin) {
		for (int i = 0; i < totalSurfs; ++i)
			if (m->surfaces[i].blendIndices && m->surfaces[i].blendWeights)
				++skinnedSurfs;
	}

	/* Pre-allocate the node array up front so animation channels (built
	   later in the bin pipeline) can reference mesh-node pointers
	   without reordering later code. */
	int playerPivots = m->is_player_bundle ? 3 : 0;
	int totalNodes = 1 + playerPivots + totalSurfs + totalTags + (hasSkin ? m->numJoints : 0);
	int firstMeshNode = 1 + playerPivots;
	int firstTagNode = firstMeshNode + totalSurfs;
	int firstJointNode = firstTagNode + totalTags;
	cgltf_node *nodes = (cgltf_node *)mc_calloc((size_t)totalNodes, sizeof(cgltf_node));

	cgltf_data data;
	memset(&data, 0, sizeof(data));
	data.asset.version = dup_str("2.0");
	data.asset.generator = dup_str("WoP modelconverter");
	data.file_type = as_glb ? cgltf_file_type_glb : cgltf_file_type_gltf;

	/* Asset extras: encode the Q3 player metadata (animation.cfg,
	   sex, headoffset, part frame counts) so a glTF -> player-dir
	   round-trip preserves everything. */
	if (m->is_player_bundle || m->numAnimations > 0 || m->anim_sex || m->anim_has_headoffset || m->numSkins > 0) {
		/* Allocate dynamically so large skin lists fit. */
		size_t cap = 16384;
		for (int i = 0; i < m->numSkins; ++i)
			cap += 64 + (size_t)m->skins[i].numEntries * 192;
		char *buf = (char *)mc_malloc(cap);
		if (!buf)
			return -1;
		size_t off = 0;
		off += (size_t)snprintf(buf + off, cap - off, "{");
		int wrote = 0;
		if (m->is_player_bundle) {
			off += (size_t)snprintf(buf + off, cap - off, "\"q3_player\":true");
			wrote = 1;
		}
		if (m->is_player_bundle) {
			off += (size_t)snprintf(buf + off, cap - off,
									 "%s\"q3_part_frames\":[%d,%d,%d]",
									 wrote ? "," : "", m->part_numFrames[0], m->part_numFrames[1],
									 m->part_numFrames[2]);
			wrote = 1;
		}
		if (m->anim_sex) {
			off += (size_t)snprintf(buf + off, cap - off, "%s\"q3_sex\":\"%c\"", wrote ? "," : "",
									 m->anim_sex);
			wrote = 1;
		}
		if (m->anim_has_headoffset) {
			off += (size_t)snprintf(buf + off, cap - off,
									 "%s\"q3_headoffset\":[%g,%g,%g]", wrote ? "," : "",
									 m->anim_headoffset[0], m->anim_headoffset[1], m->anim_headoffset[2]);
			wrote = 1;
		}
		if (m->numAnimations > 0) {
			off += (size_t)snprintf(buf + off, cap - off, "%s\"q3_animations\":[", wrote ? "," : "");
			wrote = 1;
			for (int i = 0; i < m->numAnimations && off + 256 < cap; ++i) {
				const mc_animation_t *a = &m->animations[i];
				off += (size_t)snprintf(buf + off, cap - off,
										 "%s{\"name\":\"%s\",\"first\":%d,\"num\":%d,\"loop\":%d,\"fps\":%g}",
										 i ? "," : "", a->name, a->firstFrame, a->numFrames, a->loopFrames,
										 a->fps);
			}
			off += (size_t)snprintf(buf + off, cap - off, "]");
		}
		if (m->numSkins > 0) {
			off += (size_t)snprintf(buf + off, cap - off, "%s\"q3_skins\":[", wrote ? "," : "");
			wrote = 1;
			for (int i = 0; i < m->numSkins && off + 192 < cap; ++i) {
				const mc_skin_variant_t *sv = &m->skins[i];
				off += (size_t)snprintf(buf + off, cap - off,
										 "%s{\"name\":\"%s\",\"part\":\"%s\",\"entries\":[",
										 i ? "," : "", sv->name, sv->part);
				for (int j = 0; j < sv->numEntries && off + 192 < cap; ++j) {
					off += (size_t)snprintf(buf + off, cap - off,
											 "%s{\"surface\":\"%s\",\"shader\":\"%s\"}",
											 j ? "," : "", sv->entries[j].surface, sv->entries[j].shader);
				}
				off += (size_t)snprintf(buf + off, cap - off, "]}");
			}
			off += (size_t)snprintf(buf + off, cap - off, "]");
		}
		off += (size_t)snprintf(buf + off, cap - off, "}");
		data.asset.extras.data = dup_str(buf);
		free(buf);
	}

	bin_buf_t bin = {0};

	/* ------------------------------------------------------------------
	   For each surface, we need:
		 - 1 indices accessor
		 - 1 base POSITION accessor
		 - 1 NORMAL accessor (frame 0)
		 - 1 TEXCOORD_0 accessor
		 - (numFrames - 1) morph-target POSITION accessors
	   Each accessor needs its own buffer view (we use one per attribute
	   for clarity).
	   ------------------------------------------------------------------ */

	int accessorsPerSurf = 3 + 1 + (numFrames - 1); /* pos, norm, uv, idx, +morph pos */
	int viewsPerSurf = accessorsPerSurf;
	int totalAccessors = totalSurfs * accessorsPerSurf;
	int totalViews = totalSurfs * viewsPerSurf;

	/* Skin: per skinned surface a JOINTS_0 + WEIGHTS_0 accessor pair, plus
	   one shared inverseBindMatrices accessor. */
	int skinAccessorsExtra = hasSkin ? (skinnedSurfs * 2 + 1) : 0;
	int skinViewsExtra = skinAccessorsExtra;
	totalAccessors += skinAccessorsExtra;
	totalViews += skinViewsExtra;

	/* Reserve space for animation accessors/views: per anim we add one
	   shared input (time) accessor + one weights output accessor per
	   surface.  Each accessor gets its own buffer view. */
	int animAccessorsExtra = 0;
	int animViewsExtra = 0;
	if (m->numAnimations > 0 && numFrames > 1) {
		animAccessorsExtra = m->numAnimations * (1 + totalSurfs);
		/* Player bundles also animate the pivot_upper and pivot_head
		   nodes (2 nodes * 2 paths = 4 extra accessors per anim). */
		if (m->is_player_bundle)
			animAccessorsExtra += m->numAnimations * 4;
		/* Skeletal models also animate every joint with a translation
		   vec3 + rotation vec4 sampler so IQM/MDR -> glTF -> IQM/MDR
		   can round-trip the joint TRS losslessly. */
		if (hasSkin && m->jointPoses)
			animAccessorsExtra += m->numAnimations * (m->numJoints * 2);
		/* Per-frame tag animation: emit one translation + one rotation
		   sampler per tag per anim so MD3 -> glTF -> MD3 preserves
		   per-frame tag motion (weapons, attachment points, ...).
		   Skipped for player bundles - in that mode tag motion is
		   already encoded in the pivot_upper / pivot_head animation
		   tracks, and the per-part frame ranges make a flat
		   firstFrame+t indexing scheme ambiguous. */
		if (m->numTags > 0 && !m->is_player_bundle)
			animAccessorsExtra += m->numAnimations * (m->numTags * 2);
		animViewsExtra = animAccessorsExtra;
	}

	cgltf_buffer *buffers = ALLOC_ARR(cgltf_buffer, 1);
	data.buffers = buffers;
	data.buffers_count = 1;

	/* Texture set (one image+sampler+texture per unique image referenced by
	   any material - base color, normal, emissive, metallic-roughness). */
	int maxTextures = 0;
	for (int i = 0; i < totalSurfs; ++i) {
		const mc_surface_t *s = &m->surfaces[i];
		maxTextures += ((s->texture[0] || s->shader[0]) ? 1 : 0) + (s->normal_map[0] ? 1 : 0) +
					   (s->emissive_map[0] ? 1 : 0) + (s->mr_map[0] ? 1 : 0);
	}

	cgltf_buffer_view *views = ALLOC_ARR(cgltf_buffer_view, totalViews + maxTextures + animViewsExtra);
	cgltf_accessor *accessors = ALLOC_ARR(cgltf_accessor, totalAccessors + animAccessorsExtra);
	data.buffer_views = views;
	data.buffer_views_count = totalViews; /* grown by push_texture for embedded images */
	data.accessors = accessors;
	data.accessors_count = totalAccessors;

	cgltf_mesh *meshes = ALLOC_ARR(cgltf_mesh, totalSurfs);
	data.meshes = meshes;
	data.meshes_count = totalSurfs;

	/* Materials: one per surface (kept simple - duplicates accepted). */
	cgltf_material *materials = ALLOC_ARR(cgltf_material, totalSurfs);
	data.materials = materials;
	data.materials_count = totalSurfs;

	cgltf_image *images = NULL;
	cgltf_texture *textures = NULL;
	cgltf_sampler *samplers = NULL;
	if (maxTextures > 0) {
		images = ALLOC_ARR(cgltf_image, maxTextures);
		textures = ALLOC_ARR(cgltf_texture, maxTextures);
		samplers = ALLOC_ARR(cgltf_sampler, 1);
		samplers[0].mag_filter = 9729; /* GL_LINEAR */
		samplers[0].min_filter = 9987; /* LINEAR_MIPMAP_LINEAR */
		samplers[0].wrap_s = 10497;	   /* GL_REPEAT */
		samplers[0].wrap_t = 10497;
		data.images = images;
		data.images_count = 0; /* grown below */
		data.textures = textures;
		data.textures_count = 0;
		data.samplers = samplers;
		data.samplers_count = 1;
	}

	int viewIdx = 0;
	int accIdx = 0;

	for (int si = 0; si < totalSurfs; ++si) {
		const mc_surface_t *s = &m->surfaces[si];
		cgltf_mesh *mesh = &meshes[si];
		mesh->name = dup_str(s->name[0] ? s->name : "surface");

		mesh->primitives = ALLOC_ARR(cgltf_primitive, 1);
		mesh->primitives_count = 1;
		cgltf_primitive *prim = &mesh->primitives[0];
		prim->type = cgltf_primitive_type_triangles;

		/* ---------- POSITION (frame 0) ---------- */
		size_t posOfs = bin_append(&bin, s->xyz, (size_t)s->numVerts * 3 * sizeof(float));
		cgltf_buffer_view *vPos = &views[viewIdx++];
		vPos->buffer = &buffers[0];
		vPos->offset = posOfs;
		vPos->size = (size_t)s->numVerts * 3 * sizeof(float);
		vPos->type = cgltf_buffer_view_type_vertices;

		cgltf_accessor *aPos = &accessors[accIdx++];
		aPos->component_type = cgltf_component_type_r_32f;
		aPos->type = cgltf_type_vec3;
		aPos->count = (cgltf_size)s->numVerts;
		aPos->buffer_view = vPos;
		aPos->has_min = aPos->has_max = 1;
		float mins[3], maxs[3];
		mc_compute_bounds(s->xyz, s->numVerts, mins, maxs, NULL);
		for (int k = 0; k < 3; ++k) {
			aPos->min[k] = mins[k];
			aPos->max[k] = maxs[k];
		}

		/* ---------- NORMAL (frame 0) ---------- */
		size_t normOfs = bin_append(&bin, s->normal, (size_t)s->numVerts * 3 * sizeof(float));
		cgltf_buffer_view *vNorm = &views[viewIdx++];
		vNorm->buffer = &buffers[0];
		vNorm->offset = normOfs;
		vNorm->size = (size_t)s->numVerts * 3 * sizeof(float);
		vNorm->type = cgltf_buffer_view_type_vertices;

		cgltf_accessor *aNorm = &accessors[accIdx++];
		aNorm->component_type = cgltf_component_type_r_32f;
		aNorm->type = cgltf_type_vec3;
		aNorm->count = (cgltf_size)s->numVerts;
		aNorm->buffer_view = vNorm;

		/* ---------- TEXCOORD_0 ---------- */
		size_t uvOfs = bin_append(&bin, s->st, (size_t)s->numVerts * 2 * sizeof(float));
		cgltf_buffer_view *vUv = &views[viewIdx++];
		vUv->buffer = &buffers[0];
		vUv->offset = uvOfs;
		vUv->size = (size_t)s->numVerts * 2 * sizeof(float);
		vUv->type = cgltf_buffer_view_type_vertices;

		cgltf_accessor *aUv = &accessors[accIdx++];
		aUv->component_type = cgltf_component_type_r_32f;
		aUv->type = cgltf_type_vec2;
		aUv->count = (cgltf_size)s->numVerts;
		aUv->buffer_view = vUv;

		/* ---------- INDICES (uint32) ----------
		   Q3 stores triangles CW from outside; glTF expects CCW.  Build a
		   flipped copy on the fly. */
		int *flippedIdx = (int *)mc_malloc((size_t)s->numTris * 3 * sizeof(int));
		for (int t = 0; t < s->numTris; ++t) {
			flippedIdx[t * 3 + 0] = s->indices[t * 3 + 0];
			flippedIdx[t * 3 + 1] = s->indices[t * 3 + 2];
			flippedIdx[t * 3 + 2] = s->indices[t * 3 + 1];
		}
		size_t idxOfs = bin_append(&bin, flippedIdx, (size_t)s->numTris * 3 * sizeof(int));
		free(flippedIdx);
		cgltf_buffer_view *vIdx = &views[viewIdx++];
		vIdx->buffer = &buffers[0];
		vIdx->offset = idxOfs;
		vIdx->size = (size_t)s->numTris * 3 * sizeof(int);
		vIdx->type = cgltf_buffer_view_type_indices;

		cgltf_accessor *aIdx = &accessors[accIdx++];
		aIdx->component_type = cgltf_component_type_r_32u;
		aIdx->type = cgltf_type_scalar;
		aIdx->count = (cgltf_size)s->numTris * 3;
		aIdx->buffer_view = vIdx;

		/* ---------- Attributes ---------- */
		int hasBlend = hasSkin && s->blendIndices && s->blendWeights;
		int attrCount = 3 + (hasBlend ? 2 : 0);
		prim->attributes = ALLOC_ARR(cgltf_attribute, attrCount);
		prim->attributes_count = attrCount;
		prim->attributes[0].name = dup_str("POSITION");
		prim->attributes[0].type = cgltf_attribute_type_position;
		prim->attributes[0].data = aPos;
		prim->attributes[1].name = dup_str("NORMAL");
		prim->attributes[1].type = cgltf_attribute_type_normal;
		prim->attributes[1].data = aNorm;
		prim->attributes[2].name = dup_str("TEXCOORD_0");
		prim->attributes[2].type = cgltf_attribute_type_texcoord;
		prim->attributes[2].index = 0;
		prim->attributes[2].data = aUv;
		prim->indices = aIdx;

		if (hasBlend) {
			/* JOINTS_0: UBYTE x 4 (m->numJoints <= 256). */
			size_t jOfs = bin_append(&bin, s->blendIndices, (size_t)s->numVerts * 4);
			cgltf_buffer_view *vJ = &views[viewIdx++];
			vJ->buffer = &buffers[0];
			vJ->offset = jOfs;
			vJ->size = (size_t)s->numVerts * 4;
			vJ->type = cgltf_buffer_view_type_vertices;
			cgltf_accessor *aJ = &accessors[accIdx++];
			aJ->component_type = cgltf_component_type_r_8u;
			aJ->type = cgltf_type_vec4;
			aJ->count = (cgltf_size)s->numVerts;
			aJ->buffer_view = vJ;
			prim->attributes[3].name = dup_str("JOINTS_0");
			prim->attributes[3].type = cgltf_attribute_type_joints;
			prim->attributes[3].index = 0;
			prim->attributes[3].data = aJ;

			/* WEIGHTS_0: float x 4 (already normalised on import). */
			size_t wOfs = bin_append(&bin, s->blendWeights,
									 (size_t)s->numVerts * 4 * sizeof(float));
			cgltf_buffer_view *vW = &views[viewIdx++];
			vW->buffer = &buffers[0];
			vW->offset = wOfs;
			vW->size = (size_t)s->numVerts * 4 * sizeof(float);
			vW->type = cgltf_buffer_view_type_vertices;
			cgltf_accessor *aW = &accessors[accIdx++];
			aW->component_type = cgltf_component_type_r_32f;
			aW->type = cgltf_type_vec4;
			aW->count = (cgltf_size)s->numVerts;
			aW->buffer_view = vW;
			prim->attributes[4].name = dup_str("WEIGHTS_0");
			prim->attributes[4].type = cgltf_attribute_type_weights;
			prim->attributes[4].index = 0;
			prim->attributes[4].data = aW;
		}

		/* ---------- Morph targets for additional frames ---------- */
		if (numFrames > 1) {
			prim->targets = ALLOC_ARR(cgltf_morph_target, numFrames - 1);
			prim->targets_count = numFrames - 1;
			mesh->weights = ALLOC_ARR(cgltf_float, numFrames - 1);
			mesh->weights_count = numFrames - 1;

			float *delta = (float *)mc_malloc(sizeof(float) * (size_t)s->numVerts * 3);
			for (int f = 1; f < numFrames; ++f) {
				const float *cur = s->xyz + (size_t)f * s->numVerts * 3;
				float dmin[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
				float dmax[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
				for (int v = 0; v < s->numVerts; ++v) {
					for (int k = 0; k < 3; ++k) {
						float dv = cur[v * 3 + k] - s->xyz[v * 3 + k];
						delta[v * 3 + k] = dv;
						if (dv < dmin[k]) dmin[k] = dv;
						if (dv > dmax[k]) dmax[k] = dv;
					}
				}
				size_t mOfs = bin_append(&bin, delta, (size_t)s->numVerts * 3 * sizeof(float));
				cgltf_buffer_view *vM = &views[viewIdx++];
				vM->buffer = &buffers[0];
				vM->offset = mOfs;
				vM->size = (size_t)s->numVerts * 3 * sizeof(float);
				vM->type = cgltf_buffer_view_type_vertices;

				cgltf_accessor *aM = &accessors[accIdx++];
				aM->component_type = cgltf_component_type_r_32f;
				aM->type = cgltf_type_vec3;
				aM->count = (cgltf_size)s->numVerts;
				aM->buffer_view = vM;
				aM->has_min = aM->has_max = 1;
				for (int k = 0; k < 3; ++k) {
					aM->min[k] = dmin[k];
					aM->max[k] = dmax[k];
				}

				cgltf_morph_target *t = &prim->targets[f - 1];
				t->attributes = ALLOC_ARR(cgltf_attribute, 1);
				t->attributes_count = 1;
				t->attributes[0].name = dup_str("POSITION");
				t->attributes[0].type = cgltf_attribute_type_position;
				t->attributes[0].data = aM;
			}
			free(delta);
		}

		/* ---------- Material ---------- */
		cgltf_material *mat = &materials[si];
		mat->name = dup_str(s->shader[0] ? s->shader : (s->name[0] ? s->name : "material"));
		mat->has_pbr_metallic_roughness = 1;
		for (int k = 0; k < 4; ++k)
			mat->pbr_metallic_roughness.base_color_factor[k] = s->base_color[k] != 0.0f ? s->base_color[k] : 1.0f;
		mat->pbr_metallic_roughness.metallic_factor = s->metallic_factor;
		mat->pbr_metallic_roughness.roughness_factor = s->roughness_factor != 0.0f ? s->roughness_factor : 1.0f;
		for (int k = 0; k < 3; ++k)
			mat->emissive_factor[k] = s->emissive_factor[k];
		mat->double_sided = s->two_sided ? 1 : 0;
		switch (s->alpha_mode) {
		case 1:
			mat->alpha_mode = cgltf_alpha_mode_mask;
			mat->alpha_cutoff = s->alpha_cutoff > 0.0f ? s->alpha_cutoff : 0.5f;
			break;
		case 2:
			mat->alpha_mode = cgltf_alpha_mode_blend;
			break;
		default:
			mat->alpha_mode = cgltf_alpha_mode_opaque;
			mat->alpha_cutoff = 0.5f;
			break;
		}

		/* Embed the Q3 shader name and any companion-map paths in extras so
		   they survive a glTF -> MD3/MDR -> glTF round-trip. */
		if (s->shader[0] || s->normal_map[0] || s->normal_height_map[0] || s->q3_shader_body) {
			/* Body is base64-encoded so embedded braces / quotes / newlines
			   pass cleanly through the JSON string. */
			char *body_b64 = NULL;
			if (s->q3_shader_body && s->q3_shader_body[0])
				body_b64 = mc_b64_encode(s->q3_shader_body, strlen(s->q3_shader_body));
			size_t cap = MC_MAX_PATH * 4 + 128 + (body_b64 ? strlen(body_b64) + 32 : 0);
			char *buf = (char *)mc_malloc(cap);
			if (buf) {
				size_t off = 0;
				off += (size_t)snprintf(buf + off, cap - off, "{");
				int comma = 0;
#define EXTRA_STR(key, val)                                                                                            \
	do {                                                                                                               \
		if ((val) && (val)[0]) {                                                                                       \
			off += (size_t)snprintf(buf + off, cap - off, "%s\"%s\":\"%s\"", comma ? "," : "", key, val);              \
			comma = 1;                                                                                                 \
		}                                                                                                              \
	} while (0)
				EXTRA_STR("q3_shader", s->shader);
				EXTRA_STR("q3_normal", s->normal_map);
				EXTRA_STR("q3_normalheight", s->normal_height_map);
				if (body_b64)
					EXTRA_STR("q3_body_b64", body_b64);
#undef EXTRA_STR
				off += (size_t)snprintf(buf + off, cap - off, "}");
				mat->extras.data = dup_str(buf);
				free(buf);
			}
			free(body_b64);
		}

		cgltf_texture *tex =
			push_texture(&data, images, textures, samplers, s->texture[0] ? s->texture : s->shader, s->name,
						 asset_roots, as_glb, out_dir, &bin);
		if (tex)
			mat->pbr_metallic_roughness.base_color_texture.texture = tex;
		cgltf_texture *ntex =
			push_texture(&data, images, textures, samplers, s->normal_map, s->name, asset_roots, as_glb, out_dir, &bin);
		if (ntex)
			mat->normal_texture.texture = ntex;
		cgltf_texture *etex =
			push_texture(&data, images, textures, samplers, s->emissive_map, s->name, asset_roots, as_glb, out_dir, &bin);
		if (etex)
			mat->emissive_texture.texture = etex;
		cgltf_texture *mrtex =
			push_texture(&data, images, textures, samplers, s->mr_map, s->name, asset_roots, as_glb, out_dir, &bin);
		if (mrtex)
			mat->pbr_metallic_roughness.metallic_roughness_texture.texture = mrtex;
		cgltf_texture *otex =
			push_texture(&data, images, textures, samplers, s->occlusion_map, s->name, asset_roots, as_glb, out_dir, &bin);
		if (otex) {
			mat->occlusion_texture.texture = otex;
			mat->occlusion_texture.scale = s->occlusion_strength != 0.0f ? s->occlusion_strength : 1.0f;
		}

		prim->material = mat;
	}

	/* ---- IBM accessor (emitted while accIdx still points at the slot
	   reserved by skinAccessorsExtra so animations can claim the
	   subsequent indices unchanged). */
	cgltf_accessor *aIBM = NULL;
	float *ibmAbsMats = NULL; /* keep absolute matrices for skin block. */
	if (hasSkin) {
		float *absMats = (float *)mc_calloc((size_t)m->numJoints * 16, sizeof(float));
		for (int j = 0; j < m->numJoints; ++j) {
			float local[16];
			const float *t = m->joints[j].bindTrans;
			const float *q = m->joints[j].bindRot;
			const float *sc = m->joints[j].bindScale;
			float xx = q[0]*q[0], yy = q[1]*q[1], zz = q[2]*q[2];
			float xy = q[0]*q[1], xz = q[0]*q[2], yz = q[1]*q[2];
			float wx = q[3]*q[0], wy = q[3]*q[1], wz = q[3]*q[2];
			local[0]  = (1 - 2*(yy + zz)) * sc[0];
			local[1]  = 2*(xy + wz) * sc[0];
			local[2]  = 2*(xz - wy) * sc[0];
			local[3]  = 0;
			local[4]  = 2*(xy - wz) * sc[1];
			local[5]  = (1 - 2*(xx + zz)) * sc[1];
			local[6]  = 2*(yz + wx) * sc[1];
			local[7]  = 0;
			local[8]  = 2*(xz + wy) * sc[2];
			local[9]  = 2*(yz - wx) * sc[2];
			local[10] = (1 - 2*(xx + yy)) * sc[2];
			local[11] = 0;
			local[12] = t[0]; local[13] = t[1]; local[14] = t[2]; local[15] = 1;
			float *dst = &absMats[j * 16];
			int p = m->joints[j].parent;
			if (p >= 0 && p < j) {
				const float *pa = &absMats[p * 16];
				for (int r = 0; r < 4; ++r)
					for (int c = 0; c < 4; ++c) {
						float s = 0;
						for (int k = 0; k < 4; ++k) s += pa[k*4 + r] * local[c*4 + k];
						dst[c*4 + r] = s;
					}
			} else {
				memcpy(dst, local, sizeof(local));
			}
		}
		float *ibm = (float *)mc_calloc((size_t)m->numJoints * 16, sizeof(float));
		for (int j = 0; j < m->numJoints; ++j) {
			const float *M = &absMats[j * 16];
			float inv[16];
			float a00 = M[0],  a01 = M[1],  a02 = M[2],  a03 = M[3];
			float a10 = M[4],  a11 = M[5],  a12 = M[6],  a13 = M[7];
			float a20 = M[8],  a21 = M[9],  a22 = M[10], a23 = M[11];
			float a30 = M[12], a31 = M[13], a32 = M[14], a33 = M[15];
			float b00 = a00*a11 - a01*a10, b01 = a00*a12 - a02*a10;
			float b02 = a00*a13 - a03*a10, b03 = a01*a12 - a02*a11;
			float b04 = a01*a13 - a03*a11, b05 = a02*a13 - a03*a12;
			float b06 = a20*a31 - a21*a30, b07 = a20*a32 - a22*a30;
			float b08 = a20*a33 - a23*a30, b09 = a21*a32 - a22*a31;
			float b10 = a21*a33 - a23*a31, b11 = a22*a33 - a23*a32;
			float det = b00*b11 - b01*b10 + b02*b09 + b03*b08 - b04*b07 + b05*b06;
			if (fabsf(det) < 1e-12f) {
				float ident[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
				memcpy(&ibm[j*16], ident, sizeof(ident));
				continue;
			}
			float idet = 1.0f / det;
			inv[0]  = ( a11*b11 - a12*b10 + a13*b09) * idet;
			inv[1]  = (-a01*b11 + a02*b10 - a03*b09) * idet;
			inv[2]  = ( a31*b05 - a32*b04 + a33*b03) * idet;
			inv[3]  = (-a21*b05 + a22*b04 - a23*b03) * idet;
			inv[4]  = (-a10*b11 + a12*b08 - a13*b07) * idet;
			inv[5]  = ( a00*b11 - a02*b08 + a03*b07) * idet;
			inv[6]  = (-a30*b05 + a32*b02 - a33*b01) * idet;
			inv[7]  = ( a20*b05 - a22*b02 + a23*b01) * idet;
			inv[8]  = ( a10*b10 - a11*b08 + a13*b06) * idet;
			inv[9]  = (-a00*b10 + a01*b08 - a03*b06) * idet;
			inv[10] = ( a30*b04 - a31*b02 + a33*b00) * idet;
			inv[11] = (-a20*b04 + a21*b02 - a23*b00) * idet;
			inv[12] = (-a10*b09 + a11*b07 - a12*b06) * idet;
			inv[13] = ( a00*b09 - a01*b07 + a02*b06) * idet;
			inv[14] = (-a30*b03 + a31*b01 - a32*b00) * idet;
			inv[15] = ( a20*b03 - a21*b01 + a22*b00) * idet;
			memcpy(&ibm[j * 16], inv, sizeof(inv));
		}
		size_t ibmOfs = bin_append(&bin, ibm, sizeof(float) * 16 * (size_t)m->numJoints);
		free(ibm);
		ibmAbsMats = absMats; /* freed in skin block. */
		cgltf_buffer_view *vIBM = &views[viewIdx++];
		vIBM->buffer = &buffers[0];
		vIBM->offset = ibmOfs;
		vIBM->size = sizeof(float) * 16 * (size_t)m->numJoints;
		aIBM = &accessors[accIdx++];
		aIBM->component_type = cgltf_component_type_r_32f;
		aIBM->type = cgltf_type_mat4;
		aIBM->count = (cgltf_size)m->numJoints;
		aIBM->buffer_view = vIBM;
	}

	/* ---------- Animations ----------
	   Emit one cgltf_animation per Q3 anim entry, driving each mesh's
	   morph weights with step interpolation so Blender (and any other
	   glTF viewer) can show the named animations as separate actions.

	   Frame layout we have to re-resolve for player bundles:
		 - lower frames live at merged indices [0..nLower)
		 - upper frames live at merged indices [nLower..total)
		 - animation.cfg LEGS_* firstFrame is stored in the
		   "concatenated" frame numbering; the engine subtracts
		   (LEGS_WALKCR.first - TORSO_GESTURE.first) at runtime to get
		   the actual lower-MD3 frame.  We replay that subtraction
		   here. */
	if (m->numAnimations > 0 && numFrames > 1) {
		int nLower = 0;
		int legsSkip = 0;
		int lowerTorsoTagIdx = -1, upperHeadTagIdx = -1;
		if (m->is_player_bundle) {
			nLower = m->part_numFrames[2];
			int legsFirst = -1, torsoFirst = -1;
			for (int ai = 0; ai < m->numAnimations; ++ai) {
				if (!strcmp(m->animations[ai].name, "LEGS_WALKCR"))
					legsFirst = m->animations[ai].firstFrame;
				else if (!strcmp(m->animations[ai].name, "TORSO_GESTURE"))
					torsoFirst = m->animations[ai].firstFrame;
			}
			if (legsFirst >= 0 && torsoFirst >= 0)
				legsSkip = legsFirst - torsoFirst;
			/* Locate the per-frame tag rows used to drive the pivot
			   animations.  Tags are stored as
			       m->tags[frame * numTags + tagIdx]
			   so we just need the column indices. */
			for (int ti = 0; ti < m->numTags; ++ti) {
				const mc_tag_t *tg = &m->tags[ti]; /* frame 0 */
				if (lowerTorsoTagIdx < 0 && !strcmp(tg->name, "tag_torso")
					&& !strcmp(tg->part, MC_PART_LOWER))
					lowerTorsoTagIdx = ti;
				else if (upperHeadTagIdx < 0 && !strcmp(tg->name, "tag_head")
						 && !strcmp(tg->part, MC_PART_UPPER))
					upperHeadTagIdx = ti;
			}
		}
		const int extraChans = m->is_player_bundle ? 4 : 0;
		const int hasJointAnim = (hasSkin && m->jointPoses != NULL);
		const int jointChans = hasJointAnim ? (m->numJoints * 2) : 0;
		const int tagChans = (m->numTags > 0 && !m->is_player_bundle) ? (m->numTags * 2) : 0;

		cgltf_animation *anims = ALLOC_ARR(cgltf_animation, m->numAnimations);
		data.animations = anims;
		data.animations_count = m->numAnimations;

		for (int ai = 0; ai < m->numAnimations; ++ai) {
			const mc_animation_t *a = &m->animations[ai];
			cgltf_animation *anim = &anims[ai];
			anim->name = dup_str(a->name);
			/* Stash the source's per-anim metadata so the reader can
			   reconstruct the global-frame timeline (firstFrame +
			   loopFrames) and the original fps without having to rely
			   on animation.cfg.  Critical for tag-animation round-trip
			   in morph mode where each cgltf_animation otherwise looks
			   like an isolated [0..N) clip. */
			{
				char ex[160];
				snprintf(ex, sizeof(ex),
						 "{\"q3_first_frame\":%d,\"q3_loop_frames\":%d,\"q3_fps\":%g}",
						 a->firstFrame, a->loopFrames, (double)a->fps);
				anim->extras.data = dup_str(ex);
			}

			int N = a->numFrames;
			if (N < 1) N = 1;
			float fps = a->fps > 0.0f ? a->fps : 15.0f;

			/* Time accessor (shared across this anim's samplers). */
			float *times = (float *)mc_malloc(sizeof(float) * (size_t)N);
			for (int t = 0; t < N; ++t)
				times[t] = (float)t / fps;
			size_t timeOfs = bin_append(&bin, times, sizeof(float) * (size_t)N);
			free(times);
			cgltf_buffer_view *vT = &views[data.buffer_views_count++];
			vT->buffer = &buffers[0];
			vT->offset = timeOfs;
			vT->size = sizeof(float) * (size_t)N;
			cgltf_accessor *aT = &accessors[data.accessors_count++];
			aT->component_type = cgltf_component_type_r_32f;
			aT->type = cgltf_type_scalar;
			aT->count = (cgltf_size)N;
			aT->buffer_view = vT;
			aT->has_min = aT->has_max = 1;
			aT->min[0] = 0.0f;
			aT->max[0] = (float)(N - 1) / fps;

			anim->samplers = ALLOC_ARR(cgltf_animation_sampler, totalSurfs + extraChans + jointChans + tagChans);
			anim->samplers_count = totalSurfs + extraChans + jointChans + tagChans;
			anim->channels = ALLOC_ARR(cgltf_animation_channel, totalSurfs + extraChans + jointChans + tagChans);
			anim->channels_count = totalSurfs + extraChans + jointChans + tagChans;

			int W = numFrames - 1; /* morph weight count */
			float *weightBuf = (float *)mc_calloc((size_t)N * (size_t)W, sizeof(float));

			for (int si = 0; si < totalSurfs; ++si) {
				const mc_surface_t *s = &m->surfaces[si];
				const char *nm = a->name;
				int isLegs = !strncmp(nm, "LEGS_", 5);
				int isTorso = !strncmp(nm, "TORSO_", 6);
				int isBoth = !strncmp(nm, "BOTH_", 5);

				/* Reset and fill weight buffer for this surface. */
				memset(weightBuf, 0, sizeof(float) * (size_t)N * (size_t)W);
				for (int t = 0; t < N; ++t) {
					int targetFrame = -1;
					if (m->is_player_bundle) {
						const char *part = s->part;
						if (!strcmp(part, MC_PART_LOWER)) {
							if (isLegs)
								targetFrame = (a->firstFrame - legsSkip) + t;
							else if (isBoth)
								targetFrame = a->firstFrame + t;
							/* else: TORSO_*, lower stays at frame 0 */
						} else if (!strcmp(part, MC_PART_UPPER)) {
							if (isTorso || isBoth)
								targetFrame = nLower + a->firstFrame + t;
							/* else: LEGS_*, upper stays at frame 0 */
						} else {
							/* head: static across all anims */
						}
					} else {
						targetFrame = a->firstFrame + t;
					}
					if (targetFrame > 0 && (targetFrame - 1) < W)
						weightBuf[t * W + (targetFrame - 1)] = 1.0f;
				}
				size_t wOfs = bin_append(&bin, weightBuf,
										 sizeof(float) * (size_t)N * (size_t)W);
				cgltf_buffer_view *vW = &views[data.buffer_views_count++];
				vW->buffer = &buffers[0];
				vW->offset = wOfs;
				vW->size = sizeof(float) * (size_t)N * (size_t)W;
				cgltf_accessor *aW = &accessors[data.accessors_count++];
				aW->component_type = cgltf_component_type_r_32f;
				aW->type = cgltf_type_scalar;
				aW->count = (cgltf_size)N * (cgltf_size)W;
				aW->buffer_view = vW;

				cgltf_animation_sampler *samp = &anim->samplers[si];
				samp->input = aT;
				samp->output = aW;
				samp->interpolation = cgltf_interpolation_type_step;

				cgltf_animation_channel *chan = &anim->channels[si];
				chan->sampler = samp;
				chan->target_node = &nodes[firstMeshNode + si];
				chan->target_path = cgltf_animation_path_type_weights;
			}
			free(weightBuf);

			/* ---- Pivot transform animations (player bundles) ----
			   The lower part's tag_torso animates per lower-frame
			   and drives pivot_upper, while the upper part's tag_head
			   animates per upper-frame and drives pivot_head.  Build
			   per-sample translation+rotation buffers using exactly
			   the same frame schedule as the morph weights above so
			   children visually stay attached. */
			if (m->is_player_bundle) {
				/* nodes[1]=pivot_lower, [2]=pivot_upper, [3]=pivot_head
				   (set up later in the Nodes section, but the slots
				   were reserved up front so the channel pointers
				   stay valid). */
				cgltf_node *pUpper = &nodes[2];
				cgltf_node *pHead = &nodes[3];
				const char *nm = a->name;
				int isLegs = !strncmp(nm, "LEGS_", 5);
				int isTorso = !strncmp(nm, "TORSO_", 6);
				int isBoth = !strncmp(nm, "BOTH_", 5);

				float *lowerT = (float *)mc_calloc((size_t)N * 3, sizeof(float));
				float *lowerR = (float *)mc_calloc((size_t)N * 4, sizeof(float));
				float *upperT = (float *)mc_calloc((size_t)N * 3, sizeof(float));
				float *upperR = (float *)mc_calloc((size_t)N * 4, sizeof(float));

				for (int t = 0; t < N; ++t) {
					/* Lower frame for this sample (drives pivot_upper). */
					int lowerFrame = 0;
					if (isLegs)
						lowerFrame = (a->firstFrame - legsSkip) + t;
					else if (isBoth)
						lowerFrame = a->firstFrame + t;
					if (lowerFrame < 0)
						lowerFrame = 0;
					if (lowerFrame >= nLower && nLower > 0)
						lowerFrame = nLower - 1;

					/* Upper frame (drives pivot_head).  In merged
					   indexing upper frames live at [nLower..),
					   but inside m->tags rows for the upper part are
					   the same offset since tags are stored frame-
					   contiguous across the whole bundle. */
					int upperFrame = nLower; /* upper frame 0 in merged numbering */
					if (isTorso || isBoth)
						upperFrame = nLower + a->firstFrame + t;
					int upperMax = nLower + m->part_numFrames[1] - 1;
					if (upperFrame < nLower)
						upperFrame = nLower;
					if (upperFrame > upperMax && upperMax >= nLower)
						upperFrame = upperMax;

					if (lowerTorsoTagIdx >= 0 && lowerFrame < m->numFrames) {
						const mc_tag_t *tg =
							&m->tags[(size_t)lowerFrame * m->numTags + lowerTorsoTagIdx];
						lowerT[t * 3 + 0] = tg->origin[0];
						lowerT[t * 3 + 1] = tg->origin[1];
						lowerT[t * 3 + 2] = tg->origin[2];
						q3_axis_to_quat(tg->axis, &lowerR[t * 4]);
					} else {
						lowerR[t * 4 + 3] = 1.0f;
					}
					if (upperHeadTagIdx >= 0 && upperFrame < m->numFrames) {
						const mc_tag_t *tg =
							&m->tags[(size_t)upperFrame * m->numTags + upperHeadTagIdx];
						upperT[t * 3 + 0] = tg->origin[0];
						upperT[t * 3 + 1] = tg->origin[1];
						upperT[t * 3 + 2] = tg->origin[2];
						q3_axis_to_quat(tg->axis, &upperR[t * 4]);
					} else {
						upperR[t * 4 + 3] = 1.0f;
					}
				}

				/* Helper macro: write a vec3/vec4 sampler+channel. */
				struct {
					float *buf;
					size_t comp;	/* 3 or 4 */
					cgltf_type type;
					cgltf_node *target;
					cgltf_animation_path_type path;
				} streams[4] = {
					{lowerT, 3, cgltf_type_vec3, pUpper, cgltf_animation_path_type_translation},
					{lowerR, 4, cgltf_type_vec4, pUpper, cgltf_animation_path_type_rotation},
					{upperT, 3, cgltf_type_vec3, pHead,  cgltf_animation_path_type_translation},
					{upperR, 4, cgltf_type_vec4, pHead,  cgltf_animation_path_type_rotation},
				};
				for (int ci = 0; ci < 4; ++ci) {
					size_t bytes = sizeof(float) * (size_t)N * streams[ci].comp;
					size_t ofs = bin_append(&bin, streams[ci].buf, bytes);
					cgltf_buffer_view *vV = &views[data.buffer_views_count++];
					vV->buffer = &buffers[0];
					vV->offset = ofs;
					vV->size = bytes;
					cgltf_accessor *aV = &accessors[data.accessors_count++];
					aV->component_type = cgltf_component_type_r_32f;
					aV->type = streams[ci].type;
					aV->count = (cgltf_size)N;
					aV->buffer_view = vV;

					int slot = totalSurfs + ci;
					cgltf_animation_sampler *samp = &anim->samplers[slot];
					samp->input = aT;
					samp->output = aV;
					/* Use linear interpolation for smooth pivot
					   motion - the keyframes are already at the
					   native fps so this matches what the engine
					   draws between frame samples. */
					samp->interpolation = cgltf_interpolation_type_linear;

					cgltf_animation_channel *chan = &anim->channels[slot];
					chan->sampler = samp;
					chan->target_node = streams[ci].target;
					chan->target_path = streams[ci].path;
				}
				free(lowerT); free(lowerR); free(upperT); free(upperR);
			}

			/* ---- Per-joint TRS samplers (skeletal animation) ----
			   Sample the model's per-frame jointPoses for the frames
			   covered by this animation entry and emit one translation
			   (vec3) + one rotation (vec4) channel per joint targeting
			   that joint's node.  This makes IQM/MDR -> glTF -> IQM/MDR
			   round-trips preserve every joint's TRS exactly.  Scale is
			   intentionally omitted: Q3 source formats use uniform-1
			   scale, so emitting it would just bloat the file. */
			if (hasJointAnim) {
				int firstFrame = a->firstFrame;
				if (firstFrame < 0) firstFrame = 0;
				if (firstFrame + N > m->numFrames) {
					int clamped = m->numFrames - firstFrame;
					if (clamped < 1) clamped = 1;
					(void)clamped;
				}
				for (int j = 0; j < m->numJoints; ++j) {
					float *transBuf = (float *)mc_malloc(sizeof(float) * (size_t)N * 3);
					float *rotBuf   = (float *)mc_malloc(sizeof(float) * (size_t)N * 4);
					for (int t = 0; t < N; ++t) {
						int f = firstFrame + t;
						if (f >= m->numFrames) f = m->numFrames - 1;
						if (f < 0) f = 0;
						const mc_joint_pose_t *jp =
							&m->jointPoses[(size_t)f * m->numJoints + j];
						transBuf[t * 3 + 0] = jp->trans[0];
						transBuf[t * 3 + 1] = jp->trans[1];
						transBuf[t * 3 + 2] = jp->trans[2];
						rotBuf[t * 4 + 0] = jp->rot[0];
						rotBuf[t * 4 + 1] = jp->rot[1];
						rotBuf[t * 4 + 2] = jp->rot[2];
						rotBuf[t * 4 + 3] = jp->rot[3];
					}
					struct {
						const float *buf;
						size_t comp;
						cgltf_type type;
						cgltf_animation_path_type path;
					} jstreams[2] = {
						{transBuf, 3, cgltf_type_vec3, cgltf_animation_path_type_translation},
						{rotBuf,   4, cgltf_type_vec4, cgltf_animation_path_type_rotation},
					};
					for (int ci = 0; ci < 2; ++ci) {
						size_t bytes = sizeof(float) * (size_t)N * jstreams[ci].comp;
						size_t ofs = bin_append(&bin, jstreams[ci].buf, bytes);
						cgltf_buffer_view *vV = &views[data.buffer_views_count++];
						vV->buffer = &buffers[0];
						vV->offset = ofs;
						vV->size = bytes;
						cgltf_accessor *aV = &accessors[data.accessors_count++];
						aV->component_type = cgltf_component_type_r_32f;
						aV->type = jstreams[ci].type;
						aV->count = (cgltf_size)N;
						aV->buffer_view = vV;

						int slot = totalSurfs + extraChans + j * 2 + ci;
						cgltf_animation_sampler *samp = &anim->samplers[slot];
						samp->input = aT;
						samp->output = aV;
						samp->interpolation = cgltf_interpolation_type_linear;

						cgltf_animation_channel *chan = &anim->channels[slot];
						chan->sampler = samp;
						chan->target_node = &nodes[firstJointNode + j];
						chan->target_path = jstreams[ci].path;
					}
					free(transBuf);
					free(rotBuf);
				}
			}

			/* ---- Per-tag TRS samplers (per-frame tag motion) ----
			   Q3 MD3 stores axis+origin per tag per frame; encode as
			   translation (vec3) + rotation (vec4) channels targeting
			   each tag node so MD3 -> glTF -> MD3 preserves moving
			   weapon attachment points etc.  These are pure metadata
			   tracks; tag nodes have no children so animating them is
			   transparent to viewers. */
			if (tagChans > 0) {
				int firstFrame = a->firstFrame;
				if (firstFrame < 0) firstFrame = 0;
				for (int ti = 0; ti < m->numTags; ++ti) {
					float *transBuf = (float *)mc_malloc(sizeof(float) * (size_t)N * 3);
					float *rotBuf   = (float *)mc_malloc(sizeof(float) * (size_t)N * 4);
					for (int t = 0; t < N; ++t) {
						int f = firstFrame + t;
						if (f >= m->numFrames) f = m->numFrames - 1;
						if (f < 0) f = 0;
						const mc_tag_t *tg =
							&m->tags[(size_t)f * m->numTags + ti];
						transBuf[t * 3 + 0] = tg->origin[0];
						transBuf[t * 3 + 1] = tg->origin[1];
						transBuf[t * 3 + 2] = tg->origin[2];
						q3_axis_to_quat(tg->axis, &rotBuf[t * 4]);
					}
					struct {
						const float *buf;
						size_t comp;
						cgltf_type type;
						cgltf_animation_path_type path;
					} tstreams[2] = {
						{transBuf, 3, cgltf_type_vec3, cgltf_animation_path_type_translation},
						{rotBuf,   4, cgltf_type_vec4, cgltf_animation_path_type_rotation},
					};
					for (int ci = 0; ci < 2; ++ci) {
						size_t bytes = sizeof(float) * (size_t)N * tstreams[ci].comp;
						size_t ofs = bin_append(&bin, tstreams[ci].buf, bytes);
						cgltf_buffer_view *vV = &views[data.buffer_views_count++];
						vV->buffer = &buffers[0];
						vV->offset = ofs;
						vV->size = bytes;
						cgltf_accessor *aV = &accessors[data.accessors_count++];
						aV->component_type = cgltf_component_type_r_32f;
						aV->type = tstreams[ci].type;
						aV->count = (cgltf_size)N;
						aV->buffer_view = vV;

						int slot = totalSurfs + extraChans + jointChans + ti * 2 + ci;
						cgltf_animation_sampler *samp = &anim->samplers[slot];
						samp->input = aT;
						samp->output = aV;
						samp->interpolation = cgltf_interpolation_type_linear;

						cgltf_animation_channel *chan = &anim->channels[slot];
						chan->sampler = samp;
						chan->target_node = &nodes[firstTagNode + ti];
						chan->target_path = tstreams[ci].path;
					}
					free(transBuf);
					free(rotBuf);
				}
			}
		}
	}

	/* ---------- Buffer ---------- */
	buffers[0].size = bin.size;
	if (!as_glb) {
		/* External .bin sibling. */
		char binPath[MC_MAX_PATH];
		mc_q_strncpy(binPath, path, sizeof(binPath));
		mc_strip_extension(binPath);
		size_t l = strlen(binPath);
		if (l + 5 < sizeof(binPath))
			memcpy(binPath + l, ".bin", 5);
		mc_write_file(binPath, bin.data, bin.size);
		buffers[0].uri = dup_str(mc_basename(binPath));
	} else {
		/* Inline buffer for glb output: cgltf reads data->bin / data->bin_size
		   for the BIN chunk, and ignores per-buffer data pointers. */
		data.bin = bin.data;
		data.bin_size = bin.size;
	}

	/* ---------- Nodes & scene ---------- */
	data.nodes = nodes;
	data.nodes_count = totalNodes;

	cgltf_node *root = &nodes[0];
	root->name = dup_str("root");
	/* Q3 uses +Z up while glTF uses +Y up.  Apply a -90deg rotation
	   around the X axis on the root so the model stands upright in
	   Blender, three.js, gltfpack, etc.  The reader undoes this when
	   it sees the same matrix. */
	root->has_matrix = 1;
	const float zup_to_yup[16] = {
		1, 0,  0, 0,
		0, 0, -1, 0,
		0, 1,  0, 0,
		0, 0,  0, 1,
	};
	memcpy(root->matrix, zup_to_yup, sizeof(zup_to_yup));

	/* For player bundles we set up a 3-deep pivot chain so the parts
	   line up the same way the engine renders them at runtime:
		 root
		   pivot_lower (identity)
			 lower meshes/tags
			 pivot_upper (= lower.tag_torso[frame0])
			   upper meshes/tags
			   pivot_head (= upper.tag_head[frame0])
				 head mesh/tags
	   Mesh node local matrices stay identity, and surface vertex data
	   stays in part-local space; the reader (when it sees the
	   q3_pivot extras) skips these intermediate transforms when reading
	   vertices and tags so the round-trip is lossless. */
	cgltf_node *pivotLower = NULL, *pivotUpper = NULL, *pivotHead = NULL;
	if (m->is_player_bundle) {
		pivotLower = &nodes[1];
		pivotUpper = &nodes[2];
		pivotHead = &nodes[3];

		pivotLower->name = dup_str("pivot_lower");
		pivotLower->extras.data = dup_str("{\"q3_pivot\":\"lower\"}");

		pivotUpper->name = dup_str("pivot_upper");
		pivotUpper->extras.data = dup_str("{\"q3_pivot\":\"upper\"}");
		pivotUpper->has_translation = 1;
		pivotUpper->has_rotation = 1;

		pivotHead->name = dup_str("pivot_head");
		pivotHead->extras.data = dup_str("{\"q3_pivot\":\"head\"}");
		pivotHead->has_translation = 1;
		pivotHead->has_rotation = 1;

		/* Look up the bind-pose tag transforms used as joints. */
		const mc_tag_t *lowerTorso = NULL, *upperHead = NULL;
		if (m->numTags > 0 && m->numFrames > 0) {
			for (int t = 0; t < m->numTags; ++t) {
				const mc_tag_t *tg = &m->tags[t]; /* frame 0 */
				if (!strcmp(tg->name, "tag_torso") && !strcmp(tg->part, MC_PART_LOWER))
					lowerTorso = tg;
				else if (!strcmp(tg->name, "tag_head") && !strcmp(tg->part, MC_PART_UPPER))
					upperHead = tg;
			}
		}
		float identityQuat[4] = {0, 0, 0, 1};
		if (lowerTorso) {
			pivotUpper->translation[0] = lowerTorso->origin[0];
			pivotUpper->translation[1] = lowerTorso->origin[1];
			pivotUpper->translation[2] = lowerTorso->origin[2];
			q3_axis_to_quat(lowerTorso->axis, pivotUpper->rotation);
		} else {
			memcpy(pivotUpper->rotation, identityQuat, sizeof(identityQuat));
		}
		if (upperHead) {
			pivotHead->translation[0] = upperHead->origin[0];
			pivotHead->translation[1] = upperHead->origin[1];
			pivotHead->translation[2] = upperHead->origin[2];
			q3_axis_to_quat(upperHead->axis, pivotHead->rotation);
		} else {
			memcpy(pivotHead->rotation, identityQuat, sizeof(identityQuat));
		}
	}

	/* Per-pivot child collection (for player) or single root child list. */
	cgltf_node **lowerKids = NULL, **upperKids = NULL, **headKids = NULL;
	int lowerCnt = 0, upperCnt = 0, headCnt = 0;
	int lowerCap = totalSurfs + totalTags;
	if (m->is_player_bundle) {
		lowerKids = ALLOC_ARR(cgltf_node *, lowerCap + 1); /* +1 for pivot_upper */
		upperKids = ALLOC_ARR(cgltf_node *, lowerCap + 1); /* +1 for pivot_head */
		headKids = ALLOC_ARR(cgltf_node *, lowerCap);
	}
	cgltf_node **rootChildren = ALLOC_ARR(cgltf_node *, totalSurfs + totalTags + playerPivots);
	int rootChildCnt = 0;

	for (int si = 0; si < totalSurfs; ++si) {
		cgltf_node *node = &nodes[firstMeshNode + si];
		node->name = dup_str(m->surfaces[si].name[0] ? m->surfaces[si].name : "mesh");
		node->mesh = &meshes[si];
		if (m->surfaces[si].part[0] || m->surfaces[si].lod > 0) {
			char ex[96];
			if (m->surfaces[si].lod > 0)
				snprintf(ex, sizeof(ex), "{\"q3_part\":\"%s\",\"q3_lod\":%d}",
						 m->surfaces[si].part, m->surfaces[si].lod);
			else
				snprintf(ex, sizeof(ex), "{\"q3_part\":\"%s\"}", m->surfaces[si].part);
			node->extras.data = dup_str(ex);
		}
		if (m->is_player_bundle) {
			const char *p = m->surfaces[si].part;
			if (!strcmp(p, MC_PART_HEAD))
				headKids[headCnt++] = node;
			else if (!strcmp(p, MC_PART_UPPER))
				upperKids[upperCnt++] = node;
			else
				lowerKids[lowerCnt++] = node;
		} else {
			rootChildren[rootChildCnt++] = node;
		}
	}
	for (int ti = 0; ti < totalTags; ++ti) {
		const mc_tag_t *tag = &m->tags[ti];
		cgltf_node *node = &nodes[firstTagNode + ti];
		char nm[MC_MAX_QPATH + 8];
		/* Q3 tag names already start with "tag_"; pass through verbatim
		   unless the source forgot the prefix. */
		if (!strncmp(tag->name, "tag_", 4))
			snprintf(nm, sizeof(nm), "%s", tag->name);
		else
			snprintf(nm, sizeof(nm), "tag_%s", tag->name);
		node->name = dup_str(nm);
		/* Use TRS form so per-frame tag animation channels (translation
		   + rotation) can target this node - glTF forbids animating
		   TRS on a node that uses a matrix. */
		node->has_translation = 1;
		node->translation[0] = tag->origin[0];
		node->translation[1] = tag->origin[1];
		node->translation[2] = tag->origin[2];
		node->has_rotation = 1;
		q3_axis_to_quat(tag->axis, node->rotation);
		node->has_scale = 1;
		node->scale[0] = node->scale[1] = node->scale[2] = 1.0f;
		char ex[128];
		if (tag->part[0]) {
			if (tag->lod > 0)
				snprintf(ex, sizeof(ex), "{\"q3_tag\":true,\"q3_part\":\"%s\",\"q3_lod\":%d}", tag->part, tag->lod);
			else
				snprintf(ex, sizeof(ex), "{\"q3_tag\":true,\"q3_part\":\"%s\"}", tag->part);
		} else {
			snprintf(ex, sizeof(ex), "{\"q3_tag\":true}");
		}
		node->extras.data = dup_str(ex);
		if (m->is_player_bundle) {
			const char *p = tag->part;
			if (!strcmp(p, MC_PART_HEAD))
				headKids[headCnt++] = node;
			else if (!strcmp(p, MC_PART_UPPER))
				upperKids[upperCnt++] = node;
			else
				lowerKids[lowerCnt++] = node;
		} else {
			rootChildren[rootChildCnt++] = node;
		}
	}

	if (m->is_player_bundle) {
		upperKids[upperCnt++] = pivotHead;
		lowerKids[lowerCnt++] = pivotUpper;
		pivotHead->children = headKids;
		pivotHead->children_count = headCnt;
		pivotUpper->children = upperKids;
		pivotUpper->children_count = upperCnt;
		pivotLower->children = lowerKids;
		pivotLower->children_count = lowerCnt;
		rootChildren[rootChildCnt++] = pivotLower;
	}

	/* ---- Skin: emit one node per joint, set TRS from the joint's bind
	   pose and the skin's joints[]/inverseBindMatrices accessors.  Joint
	   nodes are parented per the j->parent chain; orphan roots become
	   children of the model root so cgltf_validator considers them
	   reachable from the scene. */
	cgltf_skin *skin = NULL;
	if (hasSkin) {
		/* Per-joint child arrays (sized by counting children). */
		int *childCount = (int *)mc_calloc((size_t)m->numJoints, sizeof(int));
		for (int j = 0; j < m->numJoints; ++j) {
			int p = m->joints[j].parent;
			if (p >= 0 && p < m->numJoints) ++childCount[p];
		}
		cgltf_node ***childArrs = (cgltf_node ***)mc_calloc((size_t)m->numJoints, sizeof(*childArrs));
		int *childFill = (int *)mc_calloc((size_t)m->numJoints, sizeof(int));
		for (int j = 0; j < m->numJoints; ++j) {
			if (childCount[j] > 0)
				childArrs[j] = ALLOC_ARR(cgltf_node *, childCount[j]);
		}

		int rootJointCount = 0;
		for (int j = 0; j < m->numJoints; ++j) {
			cgltf_node *jn = &nodes[firstJointNode + j];
			jn->name = dup_str(m->joints[j].name);
			jn->has_translation = 1;
			jn->has_rotation = 1;
			jn->has_scale = 1;
			memcpy(jn->translation, m->joints[j].bindTrans, sizeof(jn->translation));
			memcpy(jn->rotation,    m->joints[j].bindRot,   sizeof(jn->rotation));
			memcpy(jn->scale,       m->joints[j].bindScale, sizeof(jn->scale));
			int p = m->joints[j].parent;
			if (p >= 0 && p < m->numJoints) {
				childArrs[p][childFill[p]++] = jn;
			} else {
				++rootJointCount;
			}
		}
		for (int j = 0; j < m->numJoints; ++j) {
			cgltf_node *jn = &nodes[firstJointNode + j];
			if (childArrs[j]) {
				jn->children = childArrs[j];
				jn->children_count = childCount[j];
			}
		}
		/* Add joint roots as children of the model root. */
		cgltf_node **rootChildrenPlus = ALLOC_ARR(cgltf_node *, rootChildCnt + rootJointCount);
		memcpy(rootChildrenPlus, rootChildren, sizeof(cgltf_node *) * (size_t)rootChildCnt);
		int rcc = rootChildCnt;
		for (int j = 0; j < m->numJoints; ++j) {
			int p = m->joints[j].parent;
			if (p < 0 || p >= m->numJoints)
				rootChildrenPlus[rcc++] = &nodes[firstJointNode + j];
		}
		free(rootChildren);
		rootChildren = rootChildrenPlus;
		rootChildCnt = rcc;

		/* Inverse bind matrices were emitted right after the surface
		   loop and saved in aIBM (with absMats kept around for any
		   future use; freed below). */
		free(ibmAbsMats);

		skin = ALLOC_ARR(cgltf_skin, 1);
		skin->name = dup_str("skeleton");
		skin->joints_count = (cgltf_size)m->numJoints;
		skin->joints = ALLOC_ARR(cgltf_node *, m->numJoints);
		for (int j = 0; j < m->numJoints; ++j)
			skin->joints[j] = &nodes[firstJointNode + j];
		skin->inverse_bind_matrices = aIBM;
		data.skins = skin;
		data.skins_count = 1;
		/* Attach the skin to every skinned mesh node. */
		for (int si = 0; si < totalSurfs; ++si) {
			if (m->surfaces[si].blendIndices && m->surfaces[si].blendWeights)
				nodes[firstMeshNode + si].skin = skin;
		}

		free(childCount);
		free(childArrs);
		free(childFill);
	}

	root->children = rootChildren;
	root->children_count = rootChildCnt;

	cgltf_scene *scenes = ALLOC_ARR(cgltf_scene, 1);
	scenes[0].nodes = ALLOC_ARR(cgltf_node *, 1);
	scenes[0].nodes[0] = root;
	scenes[0].nodes_count = 1;
	scenes[0].name = dup_str("scene");
	data.scenes = scenes;
	data.scenes_count = 1;
	data.scene = scenes;

	cgltf_options opts = {0};
	opts.type = as_glb ? cgltf_file_type_glb : cgltf_file_type_gltf;
	cgltf_result rc = cgltf_write_file(&opts, path, &data);
	if (rc != cgltf_result_success) {
		MC_ERR("gltf: write failed (%d)\n", (int)rc);
	}

	/* For non-glb, we wrote the .bin separately; bin.data still owns memory we must free. */
	if (!as_glb) {
		free(bin.data);
	} else {
		/* For glb, cgltf_write_file streams buffers[0].data to disk; we still
		   own the allocation. */
		free(bin.data);
	}

	/* ---- Free all dynamically allocated cgltf_data members ---- */

	/* Strings on the asset. */
	free(data.asset.version);
	free(data.asset.generator);
	free(data.asset.extras.data);

	/* Per-node strings and child arrays. */
	for (cgltf_size i = 0; i < data.nodes_count; ++i) {
		free(nodes[i].name);
		free(nodes[i].extras.data);
		free(nodes[i].children);
	}
	free(nodes);

	/* Per-mesh strings and sub-arrays. */
	for (cgltf_size i = 0; i < data.meshes_count; ++i) {
		free(meshes[i].name);
		for (cgltf_size p = 0; p < meshes[i].primitives_count; ++p) {
			cgltf_primitive *pr = &meshes[i].primitives[p];
			for (cgltf_size a = 0; a < pr->attributes_count; ++a)
				free(pr->attributes[a].name);
			free(pr->attributes);
			for (cgltf_size t = 0; t < pr->targets_count; ++t) {
				for (cgltf_size a = 0; a < pr->targets[t].attributes_count; ++a)
					free(pr->targets[t].attributes[a].name);
				free(pr->targets[t].attributes);
			}
			free(pr->targets);
		}
		free(meshes[i].primitives);
		free(meshes[i].weights);
	}
	free(meshes);

	/* Per-material strings. */
	for (cgltf_size i = 0; i < data.materials_count; ++i) {
		free(materials[i].name);
		free(materials[i].extras.data);
	}
	free(materials);

	/* Per-image strings. */
	if (data.images) {
		for (cgltf_size i = 0; i < data.images_count; ++i) {
			free(images[i].name);
			free(images[i].uri);
			free(images[i].mime_type);
		}
		free(images);
	}

	/* Per-texture strings. */
	if (data.textures) {
		for (cgltf_size i = 0; i < data.textures_count; ++i)
			free(textures[i].name);
		free(textures);
	}

	free(samplers);

	/* Per-animation strings and sub-arrays. */
	if (data.animations) {
		for (cgltf_size i = 0; i < data.animations_count; ++i) {
			free(data.animations[i].name);
			free(data.animations[i].extras.data);
			free(data.animations[i].samplers);
			free(data.animations[i].channels);
		}
		free(data.animations);
	}

	/* Skin. */
	if (data.skins) {
		free(skin->name);
		free(skin->joints);
		free(skin);
	}

	/* Per-scene strings and node arrays. */
	for (cgltf_size i = 0; i < data.scenes_count; ++i) {
		free(scenes[i].name);
		free(scenes[i].nodes);
	}
	free(scenes);

	/* Per-buffer strings. */
	free(buffers[0].uri);
	free(buffers);

	free(views);
	free(accessors);

	return rc == cgltf_result_success ? 0 : -1;
}
