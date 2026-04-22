/*
===========================================================================
modelconverter - MDR reader and writer.

MDR is the Quake 3: Team Arena / EliteForce skeletal model format.  It is
fundamentally a bone-skinned format: each frame stores only a list of bone
matrices, and per-vertex weights blend them at runtime.

  MDR -> mc_model_t : we sample every frame, evaluate the skinning for
                      every vertex and bake the result as per-frame
                      xyz / normal arrays for downstream MD3-style
                      consumers.  In addition we mirror the bone
                      hierarchy into mc_model_t::joints (flattened to
                      root-level joints, since MDR bones have no parent
                      index), record per-frame TRS in jointPoses, and
                      capture up to 4 blend influences per vertex into
                      blendIndices / blendWeights.  This means the
                      skeleton itself is preserved for an IQM or glTF
                      writer.  Tags inherit their bone's transform per
                      frame.  Only LOD 0 (highest detail) is read.

  mc_model_t -> MDR : when m->numJoints > 0 and at least one surface has
                      blend data, we re-emit the real skeleton: one MDR
                      bone per mc_joint_t, computing absolute bone
                      matrices per frame from the stored TRS poses, and
                      per vertex up to 4 weight records pointing at
                      those bones (offset = inv(absBindBone) *
                      frame0_vertex).  In that case animation and
                      skinning round-trip losslessly.  When the source
                      had no skeleton (e.g. plain MD3 input) we fall
                      back to the legacy static path: one identity bone,
                      every vertex weighted 100% to it with offset =
                      frame-0 position; numFrames identity bone matrices
                      are written so the file format stays valid.  An
                      additional bone per tag is appended in either case
                      so per-frame tag transforms survive in-engine.  A
                      warning is printed when we have to fall back to
                      static and the input had more than one frame.

The compressed MDR variant (header.ofsFrames < 0, mdrCompFrame_t) is not
supported: real-world content always uses uncompressed MDR.  Such files
are rejected at load time.
===========================================================================
*/

#include "mc_common.h"

#include <string.h>

/* ------------------------------------------------------------------ */
/* Read                                                                */
/* ------------------------------------------------------------------ */

static void mdr_xform_point(const float m[3][4], const float p[3], float out[3]) {
	out[0] = m[0][0] * p[0] + m[0][1] * p[1] + m[0][2] * p[2] + m[0][3];
	out[1] = m[1][0] * p[0] + m[1][1] * p[1] + m[1][2] * p[2] + m[1][3];
	out[2] = m[2][0] * p[0] + m[2][1] * p[1] + m[2][2] * p[2] + m[2][3];
}

static void mdr_xform_vec(const float m[3][4], const float v[3], float out[3]) {
	out[0] = m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2];
	out[1] = m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2];
	out[2] = m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2];
}

/* Decompose an MDR bone's 3x4 absolute matrix into TRS so we can mirror
   it into mc_model_t::jointPoses for round-trip through IQM / glTF. */
static void mdr_matrix_to_trs(const float m[3][4], float trans[3], float quat[4], float scale[3]) {
	trans[0] = m[0][3]; trans[1] = m[1][3]; trans[2] = m[2][3];
	float sx = sqrtf(m[0][0] * m[0][0] + m[1][0] * m[1][0] + m[2][0] * m[2][0]);
	float sy = sqrtf(m[0][1] * m[0][1] + m[1][1] * m[1][1] + m[2][1] * m[2][1]);
	float sz = sqrtf(m[0][2] * m[0][2] + m[1][2] * m[1][2] + m[2][2] * m[2][2]);
	scale[0] = sx; scale[1] = sy; scale[2] = sz;
	if (sx < 1e-8f) sx = 1; if (sy < 1e-8f) sy = 1; if (sz < 1e-8f) sz = 1;
	float r[3][3] = {
		{ m[0][0] / sx, m[0][1] / sy, m[0][2] / sz },
		{ m[1][0] / sx, m[1][1] / sy, m[1][2] / sz },
		{ m[2][0] / sx, m[2][1] / sy, m[2][2] / sz }
	};
	float tr = r[0][0] + r[1][1] + r[2][2];
	if (tr > 0.0f) {
		float s = sqrtf(tr + 1.0f) * 2.0f;
		quat[3] = 0.25f * s;
		quat[0] = (r[2][1] - r[1][2]) / s;
		quat[1] = (r[0][2] - r[2][0]) / s;
		quat[2] = (r[1][0] - r[0][1]) / s;
	} else if (r[0][0] > r[1][1] && r[0][0] > r[2][2]) {
		float s = sqrtf(1.0f + r[0][0] - r[1][1] - r[2][2]) * 2.0f;
		quat[3] = (r[2][1] - r[1][2]) / s;
		quat[0] = 0.25f * s;
		quat[1] = (r[0][1] + r[1][0]) / s;
		quat[2] = (r[0][2] + r[2][0]) / s;
	} else if (r[1][1] > r[2][2]) {
		float s = sqrtf(1.0f + r[1][1] - r[0][0] - r[2][2]) * 2.0f;
		quat[3] = (r[0][2] - r[2][0]) / s;
		quat[0] = (r[0][1] + r[1][0]) / s;
		quat[1] = 0.25f * s;
		quat[2] = (r[1][2] + r[2][1]) / s;
	} else {
		float s = sqrtf(1.0f + r[2][2] - r[0][0] - r[1][1]) * 2.0f;
		quat[3] = (r[1][0] - r[0][1]) / s;
		quat[0] = (r[0][2] + r[2][0]) / s;
		quat[1] = (r[1][2] + r[2][1]) / s;
		quat[2] = 0.25f * s;
	}
}

int mc_load_mdr(const char *path, mc_model_t *out) {
	size_t fileSize = 0;
	unsigned char *buf = mc_read_file(path, &fileSize);
	if (!buf)
		return -1;

	if (fileSize < sizeof(mc_mdrHeader_t)) {
		MC_ERR("error: '%s' is too small to be an MDR file\n", path);
		free(buf);
		return -1;
	}

	const mc_mdrHeader_t *hdr = (const mc_mdrHeader_t *)buf;
	if (hdr->ident != MDR_IDENT) {
		MC_ERR("error: '%s' is not an MDR file (bad ident)\n", path);
		free(buf);
		return -1;
	}
	if (hdr->version != MDR_VERSION) {
		MC_ERR("error: '%s' has unexpected MDR version %d (need %d)\n", path, hdr->version, MDR_VERSION);
		free(buf);
		return -1;
	}
	if (hdr->ofsFrames < 0) {
		MC_ERR("'%s': this file uses the compressed MDR frame format\n"
			"   (negative ofsFrames marks per-bone delta-compressed frames\n"
			"    used by some legacy assets). modelconverter currently\n"
			"    only reads the uncompressed variant.\n"
			"   workaround: re-export from the original source, or\n"
			"   convert via an engine that supports it (ioquake3) to MD3/IQM first.\n",
			path);
		free(buf);
		return -1;
	}
	if (hdr->numFrames < 1 || hdr->numBones < 1 || hdr->numLODs < 1) {
		MC_ERR("error: '%s' has no frames, bones or LODs\n", path);
		free(buf);
		return -1;
	}
	if ((size_t)hdr->ofsEnd > fileSize) {
		MC_ERR("error: '%s' header claims size %d > file %zu\n", path, hdr->ofsEnd, fileSize);
		free(buf);
		return -1;
	}

	out->numFrames = hdr->numFrames;
	out->numTags = hdr->numTags;
	out->fps = 15.0f;
	out->frames = (mc_frame_t *)calloc((size_t)out->numFrames, sizeof(mc_frame_t));
	if (out->numTags > 0)
		out->tags = (mc_tag_t *)calloc((size_t)out->numFrames * (size_t)out->numTags, sizeof(mc_tag_t));

	const size_t boneStride = sizeof(mc_mdrFrameHeader_t) + sizeof(mc_mdrBone_t) * (size_t)hdr->numBones;
	const unsigned char *framePtr = buf + hdr->ofsFrames;

	/* Cache pointers to each frame's bone array so we can re-evaluate them per surface vertex. */
	const mc_mdrBone_t **frameBones = (const mc_mdrBone_t **)calloc((size_t)hdr->numFrames, sizeof(*frameBones));
	if (!frameBones) {
		free(buf);
		return -1;
	}
	for (int f = 0; f < hdr->numFrames; ++f) {
		const mc_mdrFrameHeader_t *fh = (const mc_mdrFrameHeader_t *)(framePtr + (size_t)f * boneStride);
		frameBones[f] = (const mc_mdrBone_t *)((const unsigned char *)fh + sizeof(mc_mdrFrameHeader_t));
		mc_q_strncpy(out->frames[f].name, fh->name, sizeof(out->frames[f].name));
		for (int k = 0; k < 3; ++k) {
			out->frames[f].bounds[0][k] = fh->bounds[0][k];
			out->frames[f].bounds[1][k] = fh->bounds[1][k];
			out->frames[f].localOrigin[k] = fh->localOrigin[k];
		}
		out->frames[f].radius = fh->radius;
	}

	/* Mirror the bone hierarchy into mc_model_t::joints / jointPoses so a
	   round-trip through IQM (or future glTF skin) preserves the skeleton.
	   MDR bones are stored as absolute matrices with no parent index, so
	   we flatten everything to root-level joints (parent = -1).

	   Tags reference bones by index; our writer always lays them out as
	   [mesh bones | tag bones], so the lowest tag boneIndex marks where
	   the mesh skeleton ends.  Skip those tag-bones to avoid them
	   leaking into the joint table on round-trip.  Authors who really
	   wanted a joint that happens to be a tag attachment will simply
	   re-attach it on import; the geometry is still skinned correctly
	   because the surface bone references stay below numMeshBones. */
	const mc_mdrTag_t *tagsScan = (const mc_mdrTag_t *)(buf + hdr->ofsTags);
	int numMeshBones = hdr->numBones;
	for (int t = 0; t < hdr->numTags; ++t) {
		int b = tagsScan[t].boneIndex;
		if (b >= 0 && b < numMeshBones) numMeshBones = b;
	}
	if (numMeshBones < 1) numMeshBones = 1;
	/* Static MDRs (single identity bone + tag bones) must not produce a
	   joint table - there's nothing to skin.  Detect this by checking
	   whether the bone matrix is the identity in *every* frame; if any
	   frame deviates we have an animated single-bone model and must
	   keep the joint so the animation survives the round-trip. */
	int isStatic = 0;
	if (numMeshBones == 1) {
		const float eps = 1e-4f;
		isStatic = 1;
		for (int f = 0; f < hdr->numFrames && isStatic; ++f) {
			const float (*mf)[4] = (const float (*)[4])frameBones[f][0].matrix;
			if (fabsf(mf[0][0] - 1.0f) > eps || fabsf(mf[1][1] - 1.0f) > eps ||
				fabsf(mf[2][2] - 1.0f) > eps ||
				fabsf(mf[0][1]) > eps || fabsf(mf[0][2]) > eps || fabsf(mf[0][3]) > eps ||
				fabsf(mf[1][0]) > eps || fabsf(mf[1][2]) > eps || fabsf(mf[1][3]) > eps ||
				fabsf(mf[2][0]) > eps || fabsf(mf[2][1]) > eps || fabsf(mf[2][3]) > eps) {
				isStatic = 0;
			}
		}
	}
	if (!isStatic) mc_model_set_joints(out, numMeshBones);
	for (int j = 0; j < (isStatic ? 0 : numMeshBones); ++j) {
		mc_joint_t *mj = &out->joints[j];
		snprintf(mj->name, sizeof(mj->name), "bone_%d", j);
		mj->parent = -1;
		float t[3], q[4], sc[3];
		mdr_matrix_to_trs(frameBones[0][j].matrix, t, q, sc);
		memcpy(mj->bindTrans, t, sizeof(t));
		memcpy(mj->bindRot, q, sizeof(q));
		memcpy(mj->bindScale, sc, sizeof(sc));
		for (int f = 0; f < hdr->numFrames; ++f) {
			mc_joint_pose_t *jp = &out->jointPoses[(size_t)f * out->numJoints + j];
			float ft[3], fq[4], fs[3];
			mdr_matrix_to_trs(frameBones[f][j].matrix, ft, fq, fs);
			memcpy(jp->trans, ft, sizeof(ft));
			memcpy(jp->rot, fq, sizeof(fq));
			memcpy(jp->scale, fs, sizeof(fs));
		}
	}

	/* Walk every LOD chunk; surfaces from LOD0 use lod=0, etc.  Skinning,
	   tags and frames are global (already read above). */
	out->numLODs = hdr->numLODs;
	const mc_mdrLOD_t *lod = (const mc_mdrLOD_t *)(buf + hdr->ofsLODs);
	for (int lodLevel = 0; lodLevel < hdr->numLODs; ++lodLevel) {
		const mc_mdrSurface_t *surf =
			(const mc_mdrSurface_t *)((const unsigned char *)lod + lod->ofsSurfaces);

		for (int s = 0; s < lod->numSurfaces; ++s) {
			mc_surface_t *dst = mc_model_add_surface(out);
			mc_q_strncpy(dst->name, surf->name, sizeof(dst->name));
			mc_q_strncpy(dst->shader, surf->shader, sizeof(dst->shader));
			dst->lod = lodLevel;

		mc_surface_alloc(dst, surf->numVerts, surf->numTriangles, out->numFrames);
		mc_surface_alloc_blend(dst);

		/* Triangles. */
		const mc_mdrTriangle_t *tris =
			(const mc_mdrTriangle_t *)((const unsigned char *)surf + surf->ofsTriangles);
		for (int t = 0; t < surf->numTriangles; ++t) {
			dst->indices[t * 3 + 0] = tris[t].indexes[0];
			dst->indices[t * 3 + 1] = tris[t].indexes[1];
			dst->indices[t * 3 + 2] = tris[t].indexes[2];
		}

		/* Walk verts (variable sized due to per-vertex weights array). */
		const unsigned char *vp = (const unsigned char *)surf + surf->ofsVerts;
		for (int v = 0; v < surf->numVerts; ++v) {
			const mc_mdrVertexHeader_t *vh = (const mc_mdrVertexHeader_t *)vp;
			const mc_mdrWeight_t *weights =
				(const mc_mdrWeight_t *)(vp + sizeof(mc_mdrVertexHeader_t));

			dst->st[v * 2 + 0] = vh->texCoords[0];
			dst->st[v * 2 + 1] = vh->texCoords[1];

			/* Capture up to 4 blend influences (IQM cap) - keep the largest. */
			{
				int chosen[4] = { 0, 0, 0, 0 };
				float chosenW[4] = { 0, 0, 0, 0 };
				int picked = 0;
				for (int w = 0; w < vh->numWeights; ++w) {
					float wW = weights[w].boneWeight;
					int wB = weights[w].boneIndex;
					if (picked < 4) {
						chosen[picked] = wB;
						chosenW[picked] = wW;
						++picked;
					} else {
						/* Replace smallest existing weight if larger. */
						int smallest = 0;
						for (int k = 1; k < 4; ++k) if (chosenW[k] < chosenW[smallest]) smallest = k;
						if (wW > chosenW[smallest]) {
							chosen[smallest] = wB;
							chosenW[smallest] = wW;
						}
					}
				}
				float sum = chosenW[0] + chosenW[1] + chosenW[2] + chosenW[3];
				if (sum > 1e-6f) {
					for (int k = 0; k < 4; ++k) chosenW[k] /= sum;
				}
				for (int k = 0; k < 4; ++k) {
					int idx = chosen[k];
					if (idx < 0 || idx > 255) idx = 0;
					dst->blendIndices[v * 4 + k] = (unsigned char)idx;
					dst->blendWeights[v * 4 + k] = chosenW[k];
				}
			}

			for (int f = 0; f < out->numFrames; ++f) {
				float pos[3] = {0, 0, 0};
				float nrm[3] = {0, 0, 0};
				for (int w = 0; w < vh->numWeights; ++w) {
					const mc_mdrWeight_t *wt = &weights[w];
					if (wt->boneIndex < 0 || wt->boneIndex >= hdr->numBones)
						continue;
					const mc_mdrBone_t *b = &frameBones[f][wt->boneIndex];
					float p[3], n[3];
					mdr_xform_point(b->matrix, wt->offset, p);
					mdr_xform_vec(b->matrix, vh->normal, n);
					pos[0] += wt->boneWeight * p[0];
					pos[1] += wt->boneWeight * p[1];
					pos[2] += wt->boneWeight * p[2];
					nrm[0] += wt->boneWeight * n[0];
					nrm[1] += wt->boneWeight * n[1];
					nrm[2] += wt->boneWeight * n[2];
				}
				/* Re-normalize the blended normal. */
				float ln = sqrtf(nrm[0] * nrm[0] + nrm[1] * nrm[1] + nrm[2] * nrm[2]);
				if (ln > 1e-6f) {
					nrm[0] /= ln;
					nrm[1] /= ln;
					nrm[2] /= ln;
				} else {
					nrm[0] = 0.0f;
					nrm[1] = 0.0f;
					nrm[2] = 1.0f;
				}
				const size_t base = (size_t)f * (size_t)dst->numVerts * 3 + (size_t)v * 3;
				dst->xyz[base + 0] = pos[0];
				dst->xyz[base + 1] = pos[1];
				dst->xyz[base + 2] = pos[2];
				dst->normal[base + 0] = nrm[0];
				dst->normal[base + 1] = nrm[1];
				dst->normal[base + 2] = nrm[2];
			}

			vp += sizeof(mc_mdrVertexHeader_t) + sizeof(mc_mdrWeight_t) * (size_t)vh->numWeights;
		}

		surf = (const mc_mdrSurface_t *)((const unsigned char *)surf + surf->ofsEnd);
	}

		lod = (const mc_mdrLOD_t *)((const unsigned char *)lod + lod->ofsEnd);
	} /* end per-LOD loop */

	/* Tags - their world transform per frame is the bone matrix itself.
	   The tag array was already allocated above (numFrames * numTags rows). */
	const mc_mdrTag_t *tags = (const mc_mdrTag_t *)(buf + hdr->ofsTags);
	for (int t = 0; t < hdr->numTags; ++t) {
		const mc_mdrTag_t *src = &tags[t];
		for (int f = 0; f < out->numFrames; ++f) {
			mc_tag_t *tg = &out->tags[(size_t)f * (size_t)out->numTags + (size_t)t];
			mc_q_strncpy(tg->name, src->name, sizeof(tg->name));
			if (src->boneIndex < 0 || src->boneIndex >= hdr->numBones) {
				tg->axis[0][0] = tg->axis[1][1] = tg->axis[2][2] = 1.0f;
			} else {
				const mc_mdrBone_t *b = &frameBones[f][src->boneIndex];
				tg->origin[0] = b->matrix[0][3];
				tg->origin[1] = b->matrix[1][3];
				tg->origin[2] = b->matrix[2][3];
				/* MDR bone matrix is row-major 3x4 with the rotation in the
				   leading 3x3.  Forward = column 0 of that 3x3. */
				for (int r = 0; r < 3; ++r) {
					tg->axis[0][r] = b->matrix[r][0];
					tg->axis[1][r] = b->matrix[r][1];
					tg->axis[2][r] = b->matrix[r][2];
				}
			}
		}
	}

	free(frameBones);
	free(buf);

	MC_LOG("loaded mdr '%s': %d frames, %d bones, %d surfaces (LOD0), %d tags\n", path, hdr->numFrames, hdr->numBones,
		   lod->numSurfaces, hdr->numTags);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Write                                                               */
/* ------------------------------------------------------------------ */

/* Simple growing buffer used by the writer. */
typedef struct {
	unsigned char *data;
	size_t size;
	size_t cap;
} mdr_buf_t;

static void mdr_reserve(mdr_buf_t *b, size_t extra) {
	if (b->size + extra <= b->cap)
		return;
	size_t newCap = b->cap ? b->cap : 4096;
	while (newCap < b->size + extra)
		newCap *= 2;
	b->data = (unsigned char *)realloc(b->data, newCap);
	b->cap = newCap;
}

static size_t mdr_append(mdr_buf_t *b, const void *src, size_t n) {
	mdr_reserve(b, n);
	const size_t off = b->size;
	if (src)
		memcpy(b->data + off, src, n);
	else
		memset(b->data + off, 0, n);
	b->size += n;
	return off;
}

static void mdr_patch_int(mdr_buf_t *b, size_t off, int value) {
	memcpy(b->data + off, &value, sizeof(int));
}

/* Build a row-major 3x4 affine matrix from TRS. */
static void mdr_trs_to_m34(const float t[3], const float q[4], const float s[3], float m[3][4]) {
	float xx = q[0]*q[0], yy = q[1]*q[1], zz = q[2]*q[2];
	float xy = q[0]*q[1], xz = q[0]*q[2], yz = q[1]*q[2];
	float wx = q[3]*q[0], wy = q[3]*q[1], wz = q[3]*q[2];
	m[0][0] = (1.0f - 2.0f*(yy + zz)) * s[0];
	m[1][0] = (2.0f*(xy + wz)) * s[0];
	m[2][0] = (2.0f*(xz - wy)) * s[0];
	m[0][1] = (2.0f*(xy - wz)) * s[1];
	m[1][1] = (1.0f - 2.0f*(xx + zz)) * s[1];
	m[2][1] = (2.0f*(yz + wx)) * s[1];
	m[0][2] = (2.0f*(xz + wy)) * s[2];
	m[1][2] = (2.0f*(yz - wx)) * s[2];
	m[2][2] = (1.0f - 2.0f*(xx + yy)) * s[2];
	m[0][3] = t[0]; m[1][3] = t[1]; m[2][3] = t[2];
}

/* Multiply two row-major 3x4 affine matrices: out = a * b (treating each
   as a 4x4 with implicit [0,0,0,1] last row). */
static void mdr_mul_m34(const float a[3][4], const float b[3][4], float out[3][4]) {
	for (int r = 0; r < 3; ++r) {
		for (int c = 0; c < 3; ++c)
			out[r][c] = a[r][0]*b[0][c] + a[r][1]*b[1][c] + a[r][2]*b[2][c];
		out[r][3] = a[r][0]*b[0][3] + a[r][1]*b[1][3] + a[r][2]*b[2][3] + a[r][3];
	}
}

/* General-purpose affine 3x4 inverse via 3x3 cofactor expansion. */
static int mdr_inv_m34(const float m[3][4], float out[3][4]) {
	float a = m[0][0], b = m[0][1], c = m[0][2];
	float d = m[1][0], e = m[1][1], f = m[1][2];
	float g = m[2][0], h = m[2][1], i = m[2][2];
	float A =  (e*i - f*h);
	float B = -(d*i - f*g);
	float C =  (d*h - e*g);
	float det = a*A + b*B + c*C;
	if (fabsf(det) < 1e-12f)
		return -1;
	float idet = 1.0f / det;
	float inv[3][3];
	inv[0][0] =  A * idet;
	inv[0][1] = -(b*i - c*h) * idet;
	inv[0][2] =  (b*f - c*e) * idet;
	inv[1][0] =  B * idet;
	inv[1][1] =  (a*i - c*g) * idet;
	inv[1][2] = -(a*f - c*d) * idet;
	inv[2][0] =  C * idet;
	inv[2][1] = -(a*h - b*g) * idet;
	inv[2][2] =  (a*e - b*d) * idet;
	const float t[3] = { m[0][3], m[1][3], m[2][3] };
	out[0][0] = inv[0][0]; out[0][1] = inv[0][1]; out[0][2] = inv[0][2];
	out[1][0] = inv[1][0]; out[1][1] = inv[1][1]; out[1][2] = inv[1][2];
	out[2][0] = inv[2][0]; out[2][1] = inv[2][1]; out[2][2] = inv[2][2];
	out[0][3] = -(inv[0][0]*t[0] + inv[0][1]*t[1] + inv[0][2]*t[2]);
	out[1][3] = -(inv[1][0]*t[0] + inv[1][1]*t[1] + inv[1][2]*t[2]);
	out[2][3] = -(inv[2][0]*t[0] + inv[2][1]*t[1] + inv[2][2]*t[2]);
	return 0;
}

int mc_save_mdr(const char *path, const mc_model_t *m) {
	if (m->numSurfaces <= 0) {
		MC_ERR("error: cannot write empty MDR (no surfaces)\n");
		return -1;
	}

	/* A real skeletal write is possible only if we have joints, per-frame
	   poses, and at least one surface with blend data. */
	int hasSkel = (m->numJoints > 0 && m->jointPoses != NULL);
	if (hasSkel) {
		int anyBlend = 0;
		for (int i = 0; i < m->numSurfaces; ++i) {
			if (m->surfaces[i].blendIndices && m->surfaces[i].blendWeights) {
				anyBlend = 1;
				break;
			}
		}
		if (!anyBlend)
			hasSkel = 0;
	}

	if (!hasSkel && m->numFrames > 1) {
		MC_LOG("warning: mdr writer falling back to static (no skeleton/blend data); "
			   "%d frames will be emitted as identity-bone copies\n",
			   m->numFrames);
	}

	const int numFrames = m->numFrames > 0 ? m->numFrames : 1;
	const int numTags = m->numTags;
	/* Mesh bones occupy [0 .. firstTagBone), then one extra bone per tag.
	   Static path: 1 identity mesh bone.
	   Skeletal path: m->numJoints mesh bones (real skeleton). */
	const int firstTagBone = hasSkel ? m->numJoints : 1;
	const int numBones = firstTagBone + numTags;
	/* Determine LOD count from the maximum mc_surface_t.lod we see; LOD0
	   is the highest detail, larger values are progressively coarser.
	   Surfaces with .lod set to a level that has no actual surfaces are
	   simply skipped (the engine tolerates an empty LOD). */
	int numLODs = 1;
	for (int s = 0; s < m->numSurfaces; ++s) {
		int sl = m->surfaces[s].lod;
		if (sl < 0) sl = 0;
		if (sl + 1 > numLODs) numLODs = sl + 1;
	}

	/* Precompute per-frame absolute joint matrices and per-joint absolute
	   bind matrices + their inverses, used by the vertex offset encoder. */
	float (*absFrame)[4] = NULL; /* numFrames * numJoints rows of 3x4 */
	float (*invBind)[4]  = NULL; /* numJoints rows of 3x4 */
	if (hasSkel) {
		absFrame = (float (*)[4])malloc(sizeof(float[3][4]) * (size_t)numFrames * (size_t)m->numJoints);
		invBind  = (float (*)[4])malloc(sizeof(float[3][4]) * (size_t)m->numJoints);
		/* Bind: walk parents (assume parent < j). */
		float (*absBind)[3][4] = (float (*)[3][4])malloc(sizeof(float[3][4]) * (size_t)m->numJoints);
		for (int j = 0; j < m->numJoints; ++j) {
			float local[3][4];
			mdr_trs_to_m34(m->joints[j].bindTrans, m->joints[j].bindRot, m->joints[j].bindScale, local);
			int p = m->joints[j].parent;
			if (p >= 0 && p < j)
				mdr_mul_m34(absBind[p], local, absBind[j]);
			else
				memcpy(absBind[j], local, sizeof(local));
			float inv[3][4];
			if (mdr_inv_m34(absBind[j], inv) != 0) {
				/* Singular bind: fall back to identity inverse so writes still succeed. */
				memset(inv, 0, sizeof(inv));
				inv[0][0] = inv[1][1] = inv[2][2] = 1.0f;
			}
			memcpy(&invBind[(size_t)j * 3], inv, sizeof(inv));
		}
		/* Per-frame absolute matrices. */
		float (*tmp)[3][4] = (float (*)[3][4])malloc(sizeof(float[3][4]) * (size_t)m->numJoints);
		for (int f = 0; f < numFrames; ++f) {
			for (int j = 0; j < m->numJoints; ++j) {
				const mc_joint_pose_t *jp = &m->jointPoses[(size_t)f * (size_t)m->numJoints + j];
				float local[3][4];
				mdr_trs_to_m34(jp->trans, jp->rot, jp->scale, local);
				int p = m->joints[j].parent;
				if (p >= 0 && p < j)
					mdr_mul_m34(tmp[p], local, tmp[j]);
				else
					memcpy(tmp[j], local, sizeof(local));
				memcpy(&absFrame[((size_t)f * (size_t)m->numJoints + (size_t)j) * 3], tmp[j], sizeof(local));
			}
		}
		free(tmp);
		free(absBind);
	}

	mdr_buf_t buf = {0};

	/* Reserve header. */
	mc_mdrHeader_t hdr;
	memset(&hdr, 0, sizeof(hdr));
	hdr.ident = MDR_IDENT;
	hdr.version = MDR_VERSION;
	mc_q_strncpy(hdr.name, m->surfaces[0].name[0] ? m->surfaces[0].name : "model", sizeof(hdr.name));
	hdr.numFrames = numFrames;
	hdr.numBones = numBones;
	hdr.numLODs = numLODs;
	hdr.numTags = numTags;
	const size_t hdrOff = mdr_append(&buf, &hdr, sizeof(hdr));

	/* Frames - one identity bone per frame.  Use the per-frame bounds we
	   already have in mc_model_t when available, otherwise compute from
	   frame 0 surface verts. */
	const size_t framesOff = buf.size;
	for (int f = 0; f < numFrames; ++f) {
		mc_mdrFrameHeader_t fh;
		memset(&fh, 0, sizeof(fh));
		if (m->frames && f < m->numFrames) {
			for (int k = 0; k < 3; ++k) {
				fh.bounds[0][k] = m->frames[f].bounds[0][k];
				fh.bounds[1][k] = m->frames[f].bounds[1][k];
				fh.localOrigin[k] = m->frames[f].localOrigin[k];
			}
			fh.radius = m->frames[f].radius;
			mc_q_strncpy(fh.name, m->frames[f].name, sizeof(fh.name));
		} else {
			fh.bounds[0][0] = fh.bounds[0][1] = fh.bounds[0][2] = -1.0f;
			fh.bounds[1][0] = fh.bounds[1][1] = fh.bounds[1][2] = 1.0f;
			fh.radius = 1.0f;
		}
		mdr_append(&buf, &fh, sizeof(fh));

		/* Mesh bones for this frame: real skeleton when present, else a
		   single identity bone. */
		if (hasSkel) {
			for (int j = 0; j < m->numJoints; ++j) {
				mc_mdrBone_t bone;
				memcpy(bone.matrix,
					   &absFrame[((size_t)f * (size_t)m->numJoints + (size_t)j) * 3],
					   sizeof(bone.matrix));
				mdr_append(&buf, &bone, sizeof(bone));
			}
		} else {
			mc_mdrBone_t bone;
			memset(&bone, 0, sizeof(bone));
			bone.matrix[0][0] = 1.0f;
			bone.matrix[1][1] = 1.0f;
			bone.matrix[2][2] = 1.0f;
			mdr_append(&buf, &bone, sizeof(bone));
		}

		/* One bone per tag, encoding that tag's transform in the current
		   frame.  We have a tag transform for every (frame, tag) cell so
		   even the all-static-but-animated-tags case round-trips. */
		for (int t = 0; t < numTags; ++t) {
			const mc_tag_t *tg = &m->tags[(size_t)f * (size_t)numTags + (size_t)t];
			mc_mdrBone_t tb;
			memset(&tb, 0, sizeof(tb));
			/* mc_tag_t->axis is row-major: axis[0]=forward stored as the row.
			   MDR bone matrix is row-major 3x4 with rotation in [r][c] for
			   r,c in 0..2.  Forward = column 0, so column c corresponds to
			   axis[c]. */
			for (int c = 0; c < 3; ++c) {
				tb.matrix[0][c] = tg->axis[c][0];
				tb.matrix[1][c] = tg->axis[c][1];
				tb.matrix[2][c] = tg->axis[c][2];
			}
			tb.matrix[0][3] = tg->origin[0];
			tb.matrix[1][3] = tg->origin[1];
			tb.matrix[2][3] = tg->origin[2];
			mdr_append(&buf, &tb, sizeof(tb));
		}
	}

	/* One LOD chunk per detected level.  Frames + bones are global
	   (already emitted above) and shared across LODs; only the surface
	   list varies per LOD chunk.  Within a LOD we still iterate the
	   merged surface array but only emit those whose .lod matches. */
	size_t firstLodOff = 0;
	for (int lodLevel = 0; lodLevel < numLODs; ++lodLevel) {
		int lodSurfCount = 0;
		for (int s = 0; s < m->numSurfaces; ++s) {
			int sl = m->surfaces[s].lod < 0 ? 0 : m->surfaces[s].lod;
			if (sl == lodLevel) ++lodSurfCount;
		}

		const size_t lodOff = buf.size;
		if (lodLevel == 0) firstLodOff = lodOff;
		mc_mdrLOD_t lod;
		memset(&lod, 0, sizeof(lod));
		lod.numSurfaces = lodSurfCount;
		lod.ofsSurfaces = (int)sizeof(mc_mdrLOD_t);
		mdr_append(&buf, &lod, sizeof(lod));

		for (int s = 0; s < m->numSurfaces; ++s) {
			int sl = m->surfaces[s].lod < 0 ? 0 : m->surfaces[s].lod;
			if (sl != lodLevel) continue;
			const mc_surface_t *src = &m->surfaces[s];
			const size_t surfOff = buf.size;

		/* Bone references emitted at the end of the surface block:
		   skeletal -> all m->numJoints indices; static -> just bone 0. */
		const int numBoneRefs = hasSkel ? m->numJoints : 1;

		mc_mdrSurface_t surf;
		memset(&surf, 0, sizeof(surf));
		surf.ident = MDR_IDENT;
		mc_q_strncpy(surf.name, src->name, sizeof(surf.name));
		mc_q_strncpy(surf.shader, src->shader, sizeof(surf.shader));
		surf.numVerts = src->numVerts;
		surf.numTriangles = src->numTris;
		surf.numBoneReferences = numBoneRefs;
		mdr_append(&buf, &surf, sizeof(surf));

		/* Verts.  Skeletal path: emit each non-zero blend influence with
		   offset = inv(absBindBone[b]) * frame0_vertex.  Static path: a
		   single identity-bone weight with offset = frame0_vertex. */
		const size_t vertsOff = buf.size;
		const float *xyz0 = src->xyz; /* frame 0 starts at xyz */
		const float *nrm0 = src->normal;
		const int hasBlend = hasSkel && src->blendIndices && src->blendWeights;
		for (int v = 0; v < src->numVerts; ++v) {
			mc_mdrVertexHeader_t vh;
			memset(&vh, 0, sizeof(vh));
			vh.normal[0] = nrm0[v * 3 + 0];
			vh.normal[1] = nrm0[v * 3 + 1];
			vh.normal[2] = nrm0[v * 3 + 2];
			vh.texCoords[0] = src->st[v * 2 + 0];
			vh.texCoords[1] = src->st[v * 2 + 1];

			if (hasBlend) {
				/* Collect non-zero weights and renormalize to sum 1. */
				int   bIdx[4] = { 0 };
				float bWt[4]  = { 0 };
				int   nW = 0;
				float sum = 0.0f;
				for (int k = 0; k < 4; ++k) {
					float w = src->blendWeights[v * 4 + k];
					int   b = (int)src->blendIndices[v * 4 + k];
					if (w <= 0.0f) continue;
					if (b < 0 || b >= m->numJoints) continue;
					bIdx[nW] = b;
					bWt[nW]  = w;
					sum += w;
					++nW;
				}
				if (nW == 0) {
					/* Vertex has no influences at all -> attach 100% to joint 0. */
					bIdx[0] = 0;
					bWt[0]  = 1.0f;
					nW = 1;
					sum = 1.0f;
				}
				if (sum > 0.0f) {
					for (int k = 0; k < nW; ++k) bWt[k] /= sum;
				}
				vh.numWeights = nW;
				mdr_append(&buf, &vh, sizeof(vh));

				const float vx = xyz0[v * 3 + 0];
				const float vy = xyz0[v * 3 + 1];
				const float vz = xyz0[v * 3 + 2];
				for (int k = 0; k < nW; ++k) {
					const float (*ib)[4] = (const float (*)[4])&invBind[(size_t)bIdx[k] * 3];
					float ox = ib[0][0]*vx + ib[0][1]*vy + ib[0][2]*vz + ib[0][3];
					float oy = ib[1][0]*vx + ib[1][1]*vy + ib[1][2]*vz + ib[1][3];
					float oz = ib[2][0]*vx + ib[2][1]*vy + ib[2][2]*vz + ib[2][3];
					mc_mdrWeight_t wt;
					memset(&wt, 0, sizeof(wt));
					wt.boneIndex  = bIdx[k];
					wt.boneWeight = bWt[k];
					wt.offset[0]  = ox;
					wt.offset[1]  = oy;
					wt.offset[2]  = oz;
					mdr_append(&buf, &wt, sizeof(wt));
				}
			} else {
				vh.numWeights = 1;
				mdr_append(&buf, &vh, sizeof(vh));
				mc_mdrWeight_t wt;
				memset(&wt, 0, sizeof(wt));
				wt.boneIndex  = 0;
				wt.boneWeight = 1.0f;
				wt.offset[0]  = xyz0[v * 3 + 0];
				wt.offset[1]  = xyz0[v * 3 + 1];
				wt.offset[2]  = xyz0[v * 3 + 2];
				mdr_append(&buf, &wt, sizeof(wt));
			}
		}

		/* Triangles. */
		const size_t trisOff = buf.size;
		for (int t = 0; t < src->numTris; ++t) {
			mc_mdrTriangle_t tri;
			tri.indexes[0] = src->indices[t * 3 + 0];
			tri.indexes[1] = src->indices[t * 3 + 1];
			tri.indexes[2] = src->indices[t * 3 + 2];
			mdr_append(&buf, &tri, sizeof(tri));
		}

		/* Bone references. */
		const size_t brefOff = buf.size;
		for (int b = 0; b < numBoneRefs; ++b) {
			int boneRef = b;
			mdr_append(&buf, &boneRef, sizeof(int));
		}

		const size_t surfEnd = buf.size;

		/* Patch the surface header now that we know the offsets. */
		mc_mdrSurface_t *patched = (mc_mdrSurface_t *)(buf.data + surfOff);
		patched->ofsHeader = (int)hdrOff - (int)surfOff;
		patched->ofsVerts = (int)(vertsOff - surfOff);
		patched->ofsTriangles = (int)(trisOff - surfOff);
		patched->ofsBoneReferences = (int)(brefOff - surfOff);
		patched->ofsEnd = (int)(surfEnd - surfOff);
	}

	const size_t lodEnd = buf.size;
	mc_mdrLOD_t *lodPatched = (mc_mdrLOD_t *)(buf.data + lodOff);
	lodPatched->ofsEnd = (int)(lodEnd - lodOff);
	} /* end per-LOD loop */

	/* Tags - each one references its own dedicated bone (firstTagBone + tagIndex). */
	const size_t tagsOff = buf.size;
	for (int t = 0; t < numTags; ++t) {
		const mc_tag_t *src = &m->tags[t]; /* first-frame slot */
		mc_mdrTag_t tag;
		memset(&tag, 0, sizeof(tag));
		tag.boneIndex = firstTagBone + t;
		mc_q_strncpy(tag.name, src->name, sizeof(tag.name));
		mdr_append(&buf, &tag, sizeof(tag));
	}

	/* Patch header offsets. */
	mc_mdrHeader_t *hdrPatched = (mc_mdrHeader_t *)(buf.data + hdrOff);
	hdrPatched->ofsFrames = (int)framesOff;
	hdrPatched->ofsLODs = (int)firstLodOff;
	hdrPatched->ofsTags = (int)tagsOff;
	hdrPatched->ofsEnd = (int)buf.size;

	int rc = mc_write_file(path, buf.data, buf.size);
	free(buf.data);
	free(absFrame);
	free(invBind);
	if (rc == 0) {
		MC_LOG("wrote mdr '%s' (%zu bytes, %d surfaces, %d frames, %d LODs, %s)\n",
			   path, buf.size, m->numSurfaces, numFrames, numLODs,
			   hasSkel ? "skeletal" : "static");
	}
	return rc;
}
