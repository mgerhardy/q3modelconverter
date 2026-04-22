/*
===========================================================================
modelconverter - MD3 reader/writer
===========================================================================
*/

#include "mc_common.h"

/*
============
mc_load_md3

Loads a Quake 3 MD3 model.
============
*/
int mc_load_md3(const char *path, mc_model_t *out) {
	size_t size = 0;
	unsigned char *buf = mc_read_file(path, &size);
	if (!buf)
		return -1;

	if (size < sizeof(mc_md3Header_t)) {
		MC_ERR("md3: file too small\n");
		free(buf);
		return -1;
	}

	mc_md3Header_t *hdr = (mc_md3Header_t *)buf;
	if (hdr->ident != MD3_IDENT) {
		MC_ERR("md3: bad ident\n");
		free(buf);
		return -1;
	}
	if (hdr->version != MD3_VERSION) {
		MC_ERR("md3: bad version %d (expected %d)\n", hdr->version, MD3_VERSION);
		free(buf);
		return -1;
	}

	mc_model_init(out);
	out->numFrames = hdr->numFrames;

	/* Frames. */
	out->frames = (mc_frame_t *)calloc((size_t)hdr->numFrames, sizeof(mc_frame_t));
	mc_md3Frame_t *frames = (mc_md3Frame_t *)(buf + hdr->ofsFrames);
	for (int i = 0; i < hdr->numFrames; ++i) {
		out->frames[i].bounds[0][0] = frames[i].bounds[0][0];
		out->frames[i].bounds[0][1] = frames[i].bounds[0][1];
		out->frames[i].bounds[0][2] = frames[i].bounds[0][2];
		out->frames[i].bounds[1][0] = frames[i].bounds[1][0];
		out->frames[i].bounds[1][1] = frames[i].bounds[1][1];
		out->frames[i].bounds[1][2] = frames[i].bounds[1][2];
		out->frames[i].localOrigin[0] = frames[i].localOrigin[0];
		out->frames[i].localOrigin[1] = frames[i].localOrigin[1];
		out->frames[i].localOrigin[2] = frames[i].localOrigin[2];
		out->frames[i].radius = frames[i].radius;
		mc_q_strncpy(out->frames[i].name, frames[i].name, sizeof(out->frames[i].name));
	}

	/* Tags. */
	out->numTags = hdr->numTags;
	if (hdr->numTags > 0) {
		out->tags = (mc_tag_t *)calloc((size_t)hdr->numFrames * (size_t)hdr->numTags, sizeof(mc_tag_t));
		mc_md3Tag_t *tags = (mc_md3Tag_t *)(buf + hdr->ofsTags);
		for (int f = 0; f < hdr->numFrames; ++f) {
			for (int t = 0; t < hdr->numTags; ++t) {
				mc_md3Tag_t *src = &tags[f * hdr->numTags + t];
				mc_tag_t *dst = &out->tags[f * hdr->numTags + t];
				mc_q_strncpy(dst->name, src->name, sizeof(dst->name));
				memcpy(dst->origin, src->origin, sizeof(dst->origin));
				memcpy(dst->axis, src->axis, sizeof(dst->axis));
			}
		}
	}

	/* Surfaces. */
	out->numSurfaces = hdr->numSurfaces;
	out->surfaces = (mc_surface_t *)calloc((size_t)hdr->numSurfaces, sizeof(mc_surface_t));
	unsigned char *surfPtr = buf + hdr->ofsSurfaces;
	for (int i = 0; i < hdr->numSurfaces; ++i) {
		mc_md3Surface_t *surf = (mc_md3Surface_t *)surfPtr;
		if (surf->ident != MD3_SURFACE_IDENT) {
			MC_ERR("md3: surface %d has bad ident\n", i);
			free(buf);
			mc_model_free(out);
			return -1;
		}
		mc_surface_t *dst = &out->surfaces[i];
		mc_q_strncpy(dst->name, surf->name, sizeof(dst->name));

		mc_surface_alloc(dst, surf->numVerts, surf->numTriangles, hdr->numFrames);

		mc_md3Shader_t *shaders = (mc_md3Shader_t *)(surfPtr + surf->ofsShaders);
		if (surf->numShaders > 0) {
			mc_q_strncpy(dst->shader, shaders[0].name, sizeof(dst->shader));
		}

		mc_md3Triangle_t *tris = (mc_md3Triangle_t *)(surfPtr + surf->ofsTriangles);
		for (int t = 0; t < surf->numTriangles; ++t) {
			dst->indices[t * 3 + 0] = tris[t].indexes[0];
			dst->indices[t * 3 + 1] = tris[t].indexes[1];
			dst->indices[t * 3 + 2] = tris[t].indexes[2];
		}

		mc_md3St_t *st = (mc_md3St_t *)(surfPtr + surf->ofsSt);
		for (int v = 0; v < surf->numVerts; ++v) {
			dst->st[v * 2 + 0] = st[v].st[0];
			dst->st[v * 2 + 1] = st[v].st[1];
		}

		mc_md3XyzNormal_t *xyzn = (mc_md3XyzNormal_t *)(surfPtr + surf->ofsXyzNormals);
		for (int f = 0; f < hdr->numFrames; ++f) {
			for (int v = 0; v < surf->numVerts; ++v) {
				int idx = f * surf->numVerts + v;
				dst->xyz[idx * 3 + 0] = (float)xyzn[idx].xyz[0] * MD3_XYZ_SCALE;
				dst->xyz[idx * 3 + 1] = (float)xyzn[idx].xyz[1] * MD3_XYZ_SCALE;
				dst->xyz[idx * 3 + 2] = (float)xyzn[idx].xyz[2] * MD3_XYZ_SCALE;
				mc_latlong_to_normal(xyzn[idx].normal, &dst->normal[idx * 3]);
			}
		}

		surfPtr += surf->ofsEnd;
	}

	free(buf);
	return 0;
}

/*
============
mc_save_md3

Writes a Quake 3 MD3 model.  Performs MD3 limit validation and clamps frame
counts where necessary.
============
*/
int mc_save_md3(const char *path, const mc_model_t *m) {
	if (m->numSurfaces > MD3_MAX_SURFACES) {
		MC_ERR("md3: too many surfaces (%d > limit %d)\n"
			"   hint: merge surfaces that share a shader, or split this model into multiple MD3s linked by tags.\n",
			m->numSurfaces, MD3_MAX_SURFACES);
		return -1;
	}
	if (m->numTags > MD3_MAX_TAGS) {
		MC_ERR("md3: too many tags (%d > limit %d)\n"
			"   hint: drop unused tag_* helpers or move secondary attachment points into a child MD3.\n",
			m->numTags, MD3_MAX_TAGS);
		return -1;
	}
	int numFrames = m->numFrames;
	if (numFrames < 1)
		numFrames = 1;
	if (numFrames > MD3_MAX_FRAMES) {
		MC_ERR("md3: too many frames (%d > limit %d)\n"
			"   hint: bake fewer keyframes, or convert to MDR/IQM which support arbitrary frame counts (use -o foo.mdr / foo.iqm).\n",
			numFrames, MD3_MAX_FRAMES);
		return -1;
	}

	for (int i = 0; i < m->numSurfaces; ++i) {
		const mc_surface_t *s = &m->surfaces[i];
		if (s->numVerts > MD3_MAX_VERTS) {
			MC_ERR("md3: surface '%s' has too many verts (%d > limit %d)\n"
				"   hint: run --decimate <ratio> to reduce density, or split this surface in your DCC tool. Consider --gen-lods to keep LOD0 detail and add lower-poly fallbacks.\n",
				s->name, s->numVerts, MD3_MAX_VERTS);
			return -1;
		}
		if (s->numTris > MD3_MAX_TRIANGLES) {
			MC_ERR("md3: surface '%s' has too many triangles (%d > limit %d)\n"
				"   hint: try --decimate 0.5 (keep half the tris), --gen-lods 3, or split the surface; MDR/IQM (-o foo.mdr/iqm) have no per-surface tri cap.\n",
				s->name, s->numTris, MD3_MAX_TRIANGLES);
			return -1;
		}
	}

	/* Compute layout. */
	int frameSize = numFrames * sizeof(mc_md3Frame_t);
	int tagSize = numFrames * m->numTags * sizeof(mc_md3Tag_t);

	int *surfSizes = (int *)calloc((size_t)m->numSurfaces, sizeof(int));
	int totalSurfBytes = 0;
	for (int i = 0; i < m->numSurfaces; ++i) {
		const mc_surface_t *s = &m->surfaces[i];
		int hdr = sizeof(mc_md3Surface_t);
		int shaders = sizeof(mc_md3Shader_t); /* always one shader */
		int tris = s->numTris * sizeof(mc_md3Triangle_t);
		int st = s->numVerts * sizeof(mc_md3St_t);
		int xyzn = numFrames * s->numVerts * sizeof(mc_md3XyzNormal_t);
		surfSizes[i] = hdr + shaders + tris + st + xyzn;
		totalSurfBytes += surfSizes[i];
	}

	int ofsFrames = sizeof(mc_md3Header_t);
	int ofsTags = ofsFrames + frameSize;
	int ofsSurfaces = ofsTags + tagSize;
	int totalSize = ofsSurfaces + totalSurfBytes;

	unsigned char *buf = (unsigned char *)calloc(1, (size_t)totalSize);
	if (!buf) {
		free(surfSizes);
		return -1;
	}

	/* Header. */
	mc_md3Header_t *hdr = (mc_md3Header_t *)buf;
	hdr->ident = MD3_IDENT;
	hdr->version = MD3_VERSION;
	{
		const char *base = mc_basename(path);
		char tmp[MC_MAX_QPATH];
		mc_q_strncpy(tmp, base, sizeof(tmp));
		mc_strip_extension(tmp);
		mc_q_strncpy(hdr->name, tmp, sizeof(hdr->name));
	}
	hdr->flags = 0;
	hdr->numFrames = numFrames;
	hdr->numTags = m->numTags;
	hdr->numSurfaces = m->numSurfaces;
	hdr->numSkins = 0;
	hdr->ofsFrames = ofsFrames;
	hdr->ofsTags = ofsTags;
	hdr->ofsSurfaces = ofsSurfaces;
	hdr->ofsEnd = totalSize;

	/* Frames. */
	mc_md3Frame_t *outFrames = (mc_md3Frame_t *)(buf + ofsFrames);
	for (int f = 0; f < numFrames; ++f) {
		const mc_frame_t *src = (m->numFrames > 0) ? &m->frames[f] : NULL;
		float mins[3] = {1e30f, 1e30f, 1e30f};
		float maxs[3] = {-1e30f, -1e30f, -1e30f};
		float radius = 0.0f;

		if (src) {
			memcpy(mins, src->bounds[0], sizeof(mins));
			memcpy(maxs, src->bounds[1], sizeof(maxs));
			radius = src->radius;
		} else {
			/* Compute bounds from this frame's vertices across all surfaces. */
			for (int i = 0; i < m->numSurfaces; ++i) {
				const mc_surface_t *s = &m->surfaces[i];
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
			float cx = 0.5f * (mins[0] + maxs[0]);
			float cy = 0.5f * (mins[1] + maxs[1]);
			float cz = 0.5f * (mins[2] + maxs[2]);
			float r2 = 0;
			for (int i = 0; i < m->numSurfaces; ++i) {
				const mc_surface_t *s = &m->surfaces[i];
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
			radius = sqrtf(r2);
		}

		memcpy(outFrames[f].bounds[0], mins, sizeof(mins));
		memcpy(outFrames[f].bounds[1], maxs, sizeof(maxs));
		outFrames[f].localOrigin[0] = 0;
		outFrames[f].localOrigin[1] = 0;
		outFrames[f].localOrigin[2] = 0;
		outFrames[f].radius = radius;
		if (src && src->name[0]) {
			mc_q_strncpy(outFrames[f].name, src->name, sizeof(outFrames[f].name));
		} else {
			snprintf(outFrames[f].name, sizeof(outFrames[f].name), "frame%d", f);
		}
	}

	/* Tags. */
	mc_md3Tag_t *outTags = (mc_md3Tag_t *)(buf + ofsTags);
	for (int f = 0; f < numFrames; ++f) {
		for (int t = 0; t < m->numTags; ++t) {
			const mc_tag_t *src;
			if (m->numFrames > 0 && m->tags) {
				src = &m->tags[f * m->numTags + t];
			} else {
				src = &m->tags[t];
			}
			mc_md3Tag_t *dst = &outTags[f * m->numTags + t];
			mc_q_strncpy(dst->name, src->name, sizeof(dst->name));
			memcpy(dst->origin, src->origin, sizeof(dst->origin));
			memcpy(dst->axis, src->axis, sizeof(dst->axis));
		}
	}

	/* Surfaces. */
	int surfOfs = ofsSurfaces;
	for (int i = 0; i < m->numSurfaces; ++i) {
		const mc_surface_t *s = &m->surfaces[i];
		mc_md3Surface_t *surf = (mc_md3Surface_t *)(buf + surfOfs);
		surf->ident = MD3_SURFACE_IDENT;
		mc_q_strncpy(surf->name, s->name, sizeof(surf->name));
		surf->flags = 0;
		surf->numFrames = numFrames;
		surf->numShaders = 1;
		surf->numVerts = s->numVerts;
		surf->numTriangles = s->numTris;

		int relShaders = sizeof(mc_md3Surface_t);
		int relTris = relShaders + sizeof(mc_md3Shader_t);
		int relSt = relTris + s->numTris * sizeof(mc_md3Triangle_t);
		int relXyzn = relSt + s->numVerts * sizeof(mc_md3St_t);
		surf->ofsShaders = relShaders;
		surf->ofsTriangles = relTris;
		surf->ofsSt = relSt;
		surf->ofsXyzNormals = relXyzn;
		surf->ofsEnd = surfSizes[i];

		mc_md3Shader_t *outShaders = (mc_md3Shader_t *)((unsigned char *)surf + relShaders);
		mc_q_strncpy(outShaders[0].name, s->shader[0] ? s->shader : (s->texture[0] ? s->texture : ""),
					 sizeof(outShaders[0].name));
		outShaders[0].shaderIndex = 0;

		mc_md3Triangle_t *outTris = (mc_md3Triangle_t *)((unsigned char *)surf + relTris);
		for (int t = 0; t < s->numTris; ++t) {
			outTris[t].indexes[0] = s->indices[t * 3 + 0];
			outTris[t].indexes[1] = s->indices[t * 3 + 1];
			outTris[t].indexes[2] = s->indices[t * 3 + 2];
		}

		mc_md3St_t *outSt = (mc_md3St_t *)((unsigned char *)surf + relSt);
		for (int v = 0; v < s->numVerts; ++v) {
			outSt[v].st[0] = s->st[v * 2 + 0];
			outSt[v].st[1] = s->st[v * 2 + 1];
		}

		mc_md3XyzNormal_t *outXyzn = (mc_md3XyzNormal_t *)((unsigned char *)surf + relXyzn);
		for (int f = 0; f < numFrames; ++f) {
			for (int v = 0; v < s->numVerts; ++v) {
				int idx = f * s->numVerts + v;
				const float *p = &s->xyz[idx * 3];
				const float *n = &s->normal[idx * 3];
				/* Quantize position to short via the standard MD3 scale.
				   Out-of-range values are clamped to the int16 range. */
				for (int k = 0; k < 3; ++k) {
					float q = p[k] / MD3_XYZ_SCALE;
					if (q > 32767.0f)
						q = 32767.0f;
					if (q < -32768.0f)
						q = -32768.0f;
					outXyzn[idx].xyz[k] = (short)lrintf(q);
				}
				outXyzn[idx].normal = mc_normal_to_latlong(n);
			}
		}

		surfOfs += surfSizes[i];
	}

	int rc = mc_write_file(path, buf, (size_t)totalSize);
	free(buf);
	free(surfSizes);
	return rc;
}
