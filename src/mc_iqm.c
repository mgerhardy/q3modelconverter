/*
===========================================================================
modelconverter - IQM reader and writer.

Reader:
  Loads an IQM file into the in-memory generic model.  Joints, per-frame
  poses (TRS) and per-vertex BLENDINDEXES / BLENDWEIGHTS are mirrored
  into mc_model_t::joints / jointPoses / surface blend arrays so the
  skeleton round-trips losslessly to other skeletal formats (MDR, glTF
  skin).  In addition every animation frame is evaluated against the
  blend arrays to produce baked per-frame xyz / normal vertex data, so
  that purely vertex-animated consumers (MD3, glTF morph targets) can
  replay an IQM animation as well.  Animations are recorded as
  mc_animation_t entries so animation.cfg-style metadata round-trips
  through glTF and back.  When the file has no joints (a static IQM)
  the reader falls back to the classic single-frame layout.

Writer:
  Emits a fully skeletal IQM when m->numJoints > 0 and at least one
  surface carries blend data: joints, poses, framedata (10 channels per
  joint with computed channelscale / channeloffset), animations and
  BLENDINDEXES / BLENDWEIGHTS vertex arrays are all written.  When the
  source has no skeleton (e.g. plain MD3 input) the writer falls back
  to a static IQM containing just POSITION / NORMAL / TEXCOORD vertex
  arrays from frame 0, which is the minimum the renderer's IQM loader
  accepts.
===========================================================================
*/

#include "mc_common.h"
#include "iqm.h"

typedef struct {
	const unsigned char *base;
	const iqmHeader_t *hdr;
} iqm_view_t;

static const char *iqm_text(const iqm_view_t *v, unsigned int ofs) {
	if (ofs >= v->hdr->num_text)
		return "";
	return (const char *)(v->base + v->hdr->ofs_text + ofs);
}

static int iqm_array_size(unsigned int format) {
	switch (format) {
	case IQM_BYTE:
	case IQM_UBYTE:
		return 1;
	case IQM_SHORT:
	case IQM_USHORT:
	case IQM_HALF:
		return 2;
	case IQM_INT:
	case IQM_UINT:
	case IQM_FLOAT:
		return 4;
	case IQM_DOUBLE:
		return 8;
	default:
		return 0;
	}
}

static void iqm_read_float(const unsigned char *base, unsigned int format, int comp, const unsigned char *p,
						   float *out) {
	int sz = iqm_array_size(format);
	for (int i = 0; i < comp; ++i) {
		switch (format) {
		case IQM_FLOAT:
			out[i] = ((const float *)p)[i];
			break;
		case IQM_UBYTE:
			out[i] = ((const unsigned char *)p)[i] / 255.0f;
			break;
		case IQM_BYTE:
			out[i] = ((const signed char *)p)[i] / 127.0f;
			break;
		case IQM_USHORT:
			out[i] = ((const unsigned short *)p)[i] / 65535.0f;
			break;
		case IQM_SHORT:
			out[i] = ((const short *)p)[i] / 32767.0f;
			break;
		default:
			out[i] = 0.0f;
			break;
		}
	}
	(void)base;
	(void)sz;
}

/* Read up to `comp` raw uint8 values without scaling - used for
   BLENDINDEXES (and BLENDWEIGHTS when normalised to 0..1). */
static void iqm_read_ubyte(const unsigned char *p, int comp, unsigned char *out) {
	for (int i = 0; i < comp; ++i)
		out[i] = p[i];
}

/* ------------------------------------------------------------------ */
/* Tiny matrix / quaternion helpers used only by the skinning sampler.*/
/* ------------------------------------------------------------------ */

static void mat34_identity(float m[3][4]) {
	memset(m, 0, sizeof(float) * 12);
	m[0][0] = m[1][1] = m[2][2] = 1.0f;
}

static void mat34_from_trs(const float t[3], const float q[4], const float s[3], float m[3][4]) {
	/* q is (x,y,z,w). */
	float x = q[0], y = q[1], z = q[2], w = q[3];
	/* Renormalise to be safe - IQM stores quaternions as raw floats. */
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
		for (int j = 0; j < 3; ++j) {
			r[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
		}
		r[i][3] = a[i][0] * b[0][3] + a[i][1] * b[1][3] + a[i][2] * b[2][3] + a[i][3];
	}
	memcpy(out, r, sizeof(r));
}

/* Invert an affine 3x4 matrix that is the product of rotation and
   uniform-or-non-uniform scale + translation.  Falls back to identity
   for singular cases. */
static void mat34_invert(const float m[3][4], float out[3][4]) {
	float a = m[0][0], b = m[0][1], c = m[0][2];
	float d = m[1][0], e = m[1][1], f = m[1][2];
	float g = m[2][0], h = m[2][1], i = m[2][2];
	float det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
	if (fabsf(det) < 1e-10f) {
		mat34_identity(out);
		return;
	}
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

int mc_load_iqm(const char *path, mc_model_t *out) {
	size_t size = 0;
	unsigned char *buf = mc_read_file(path, &size);
	if (!buf)
		return -1;

	if (size < sizeof(iqmHeader_t)) {
		MC_ERR("iqm: file too small\n");
		free(buf);
		return -1;
	}
	iqmHeader_t *hdr = (iqmHeader_t *)buf;
	if (memcmp(hdr->magic, IQM_MAGIC, 16) != 0) {
		MC_ERR("iqm: bad magic\n");
		free(buf);
		return -1;
	}
	if (hdr->version != IQM_VERSION) {
		MC_ERR("iqm: bad version %u\n", hdr->version);
		free(buf);
		return -1;
	}

	iqm_view_t v = {buf, hdr};

	/* Validate section offsets against file size. */
	if (!mc_bounds_check(hdr->ofs_vertexarrays,
			(size_t)hdr->num_vertexarrays * sizeof(iqmVertexArray_t), size) ||
		!mc_bounds_check(hdr->ofs_meshes,
			(size_t)hdr->num_meshes * sizeof(iqmMesh_t), size) ||
		!mc_bounds_check(hdr->ofs_triangles,
			(size_t)hdr->num_triangles * sizeof(iqmTriangle_t), size) ||
		!mc_bounds_check(hdr->ofs_text, hdr->num_text, size)) {
		MC_ERR("iqm: section offsets out of bounds\n");
		free(buf);
		return -1;
	}
	if (hdr->num_joints > 0 &&
		!mc_bounds_check(hdr->ofs_joints,
			(size_t)hdr->num_joints * sizeof(iqmJoint_t), size)) {
		MC_ERR("iqm: joints offset out of bounds\n");
		free(buf);
		return -1;
	}
	if (hdr->num_poses > 0 &&
		!mc_bounds_check(hdr->ofs_poses,
			(size_t)hdr->num_poses * sizeof(iqmPose_t), size)) {
		MC_ERR("iqm: poses offset out of bounds\n");
		free(buf);
		return -1;
	}
	if (hdr->num_anims > 0 &&
		!mc_bounds_check(hdr->ofs_anims,
			(size_t)hdr->num_anims * sizeof(iqmAnim_t), size)) {
		MC_ERR("iqm: anims offset out of bounds\n");
		free(buf);
		return -1;
	}
	if (hdr->num_frames > 0 && hdr->num_framechannels > 0 &&
		!mc_bounds_check(hdr->ofs_frames,
			(size_t)hdr->num_frames * (size_t)hdr->num_framechannels * sizeof(unsigned short), size)) {
		MC_ERR("iqm: frame data offset out of bounds\n");
		free(buf);
		return -1;
	}

	/* Locate vertex arrays for POSITION, NORMAL, TEXCOORD, BLENDINDEXES, BLENDWEIGHTS. */
	const iqmVertexArray_t *vas = (const iqmVertexArray_t *)(buf + hdr->ofs_vertexarrays);
	const unsigned char *posPtr = NULL, *normPtr = NULL, *uvPtr = NULL;
	const unsigned char *biPtr = NULL, *bwPtr = NULL;
	unsigned int posFmt = 0, normFmt = 0, uvFmt = 0, biFmt = 0, bwFmt = 0;
	int posComp = 0, normComp = 0, uvComp = 0, biComp = 0, bwComp = 0;
	for (unsigned int i = 0; i < hdr->num_vertexarrays; ++i) {
		const iqmVertexArray_t *va = &vas[i];
		int elemSz = iqm_array_size(va->format);
		if (elemSz <= 0) continue;
		size_t arrayBytes = (size_t)hdr->num_vertexes * (size_t)va->size * (size_t)elemSz;
		if (!mc_bounds_check(va->offset, arrayBytes, size)) {
			MC_ERR("iqm: vertex array %u offset out of bounds\n", i);
			free(buf);
			return -1;
		}
		const unsigned char *p = buf + va->offset;
		switch (va->type) {
		case IQM_POSITION:
			posPtr = p; posFmt = va->format; posComp = (int)va->size; break;
		case IQM_NORMAL:
			normPtr = p; normFmt = va->format; normComp = (int)va->size; break;
		case IQM_TEXCOORD:
			uvPtr = p; uvFmt = va->format; uvComp = (int)va->size; break;
		case IQM_BLENDINDEXES:
			biPtr = p; biFmt = va->format; biComp = (int)va->size; break;
		case IQM_BLENDWEIGHTS:
			bwPtr = p; bwFmt = va->format; bwComp = (int)va->size; break;
		default:
			break;
		}
	}
	if (!posPtr) {
		MC_ERR("iqm: no POSITION array\n");
		free(buf);
		return -1;
	}

	/* Determine how many output frames we need.  Static IQMs (no joints
	   or no animation frames) collapse to a single frame.  Skinned IQMs
	   produce one frame per IQM frame so animation is captured as
	   per-vertex morph targets downstream. */
	int numFrames = 1;
	int isSkinned = (hdr->num_joints > 0 && hdr->num_poses > 0 && hdr->num_frames > 0 && biPtr && bwPtr);
	if (isSkinned)
		numFrames = (int)hdr->num_frames;

	mc_model_init(out);
	for (int f = 0; f < numFrames; ++f)
		mc_model_add_frame(out);
	out->fps = 15.0f;

	/* Read joints (bind pose) and compute absolute bind matrices and their inverses. */
	float (*bindAbs)[3][4] = NULL;
	float (*bindAbsInv)[3][4] = NULL;
	const iqmJoint_t *joints = NULL;
	const iqmPose_t *poses = NULL;
	if (isSkinned) {
		joints = (const iqmJoint_t *)(buf + hdr->ofs_joints);
		poses = (const iqmPose_t *)(buf + hdr->ofs_poses);
		bindAbs = (float (*)[3][4])mc_calloc(hdr->num_joints, sizeof(*bindAbs));
		bindAbsInv = (float (*)[3][4])mc_calloc(hdr->num_joints, sizeof(*bindAbsInv));
		for (unsigned int j = 0; j < hdr->num_joints; ++j) {
			float local[3][4];
			mat34_from_trs(joints[j].translate, joints[j].rotate, joints[j].scale, local);
			if (joints[j].parent >= 0)
				mat34_mul(bindAbs[joints[j].parent], local, bindAbs[j]);
			else
				memcpy(bindAbs[j], local, sizeof(local));
			mat34_invert(bindAbs[j], bindAbsInv[j]);
		}

		/* Mirror the IQM skeleton into mc_model_t so writers can emit it. */
		mc_model_set_joints(out, (int)hdr->num_joints);
		for (unsigned int j = 0; j < hdr->num_joints; ++j) {
			mc_joint_t *mj = &out->joints[j];
			mc_q_strncpy(mj->name, iqm_text(&v, joints[j].name), sizeof(mj->name));
			mj->parent = joints[j].parent;
			memcpy(mj->bindTrans, joints[j].translate, sizeof(mj->bindTrans));
			memcpy(mj->bindRot,   joints[j].rotate,    sizeof(mj->bindRot));
			memcpy(mj->bindScale, joints[j].scale,     sizeof(mj->bindScale));
			/* Frame 0 = bind pose by default; later frames are overwritten
			   from the channel stream. */
			for (int f = 0; f < numFrames; ++f) {
				mc_joint_pose_t *jp = &out->jointPoses[(size_t)f * out->numJoints + j];
				memcpy(jp->trans, joints[j].translate, sizeof(jp->trans));
				memcpy(jp->rot,   joints[j].rotate,    sizeof(jp->rot));
				memcpy(jp->scale, joints[j].scale,     sizeof(jp->scale));
			}
		}

		/* Sample IQM anim entries into mc_animation_t for round-trip. */
		if (hdr->num_anims > 0) {
			const iqmAnim_t *anims = (const iqmAnim_t *)(buf + hdr->ofs_anims);
			for (unsigned int a = 0; a < hdr->num_anims; ++a) {
				mc_animation_t *am = mc_model_add_animation(out);
				const char *nm = iqm_text(&v, anims[a].name);
				mc_q_strncpy(am->name, nm[0] ? nm : "anim", sizeof(am->name));
				am->firstFrame = (int)anims[a].first_frame;
				am->numFrames = (int)anims[a].num_frames;
				am->loopFrames = (anims[a].flags & IQM_LOOP) ? am->numFrames : 0;
				am->fps = anims[a].framerate > 0 ? anims[a].framerate : 15.0f;
			}
			out->fps = anims[0].framerate > 0 ? anims[0].framerate : 15.0f;
		}
	}

	/* Per-frame absolute joint matrices and skinning matrices. */
	float (*frameSkin)[3][4] = NULL;
	if (isSkinned) {
		frameSkin = (float (*)[3][4])mc_calloc(hdr->num_joints, sizeof(*frameSkin));
	}

	const iqmMesh_t *meshes = (const iqmMesh_t *)(buf + hdr->ofs_meshes);
	const iqmTriangle_t *tris = (const iqmTriangle_t *)(buf + hdr->ofs_triangles);

	int posStride = posComp * iqm_array_size(posFmt);
	int normStride = normComp * iqm_array_size(normFmt);
	int uvStride = uvComp * iqm_array_size(uvFmt);
	int biStride = biPtr ? biComp * iqm_array_size(biFmt) : 0;
	int bwStride = bwPtr ? bwComp * iqm_array_size(bwFmt) : 0;
	const unsigned short *frameData = (isSkinned) ? (const unsigned short *)(buf + hdr->ofs_frames) : NULL;

	for (unsigned int mi = 0; mi < hdr->num_meshes; ++mi) {
		const iqmMesh_t *mesh = &meshes[mi];
		mc_surface_t *s = mc_model_add_surface(out);
		mc_q_strncpy(s->name, iqm_text(&v, mesh->name), sizeof(s->name));
		mc_q_strncpy(s->shader, iqm_text(&v, mesh->material), sizeof(s->shader));

		mc_surface_alloc(s, (int)mesh->num_vertexes, (int)mesh->num_triangles, numFrames);

		/* Bind-pose vertex data (frame 0 source). */
		float *bindPos = (float *)mc_calloc((size_t)mesh->num_vertexes * 3, sizeof(float));
		float *bindNrm = (float *)mc_calloc((size_t)mesh->num_vertexes * 3, sizeof(float));
		unsigned char *bIdx = NULL;
		float *bWgt = NULL;
		if (isSkinned) {
			bIdx = (unsigned char *)mc_calloc((size_t)mesh->num_vertexes * 4, 1);
			bWgt = (float *)mc_calloc((size_t)mesh->num_vertexes * 4, sizeof(float));
		}

		for (unsigned int vi = 0; vi < mesh->num_vertexes; ++vi) {
			unsigned int gi = mesh->first_vertex + vi;
			float p[4] = {0, 0, 0, 0};
			iqm_read_float(buf, posFmt, posComp, posPtr + (size_t)gi * posStride, p);
			bindPos[vi * 3 + 0] = p[0];
			bindPos[vi * 3 + 1] = p[1];
			bindPos[vi * 3 + 2] = p[2];

			if (normPtr) {
				float n[4] = {0, 0, 1, 0};
				iqm_read_float(buf, normFmt, normComp, normPtr + (size_t)gi * normStride, n);
				bindNrm[vi * 3 + 0] = n[0];
				bindNrm[vi * 3 + 1] = n[1];
				bindNrm[vi * 3 + 2] = n[2];
			} else {
				bindNrm[vi * 3 + 2] = 1.0f;
			}

			if (uvPtr) {
				float t[4] = {0, 0, 0, 0};
				iqm_read_float(buf, uvFmt, uvComp, uvPtr + (size_t)gi * uvStride, t);
				s->st[vi * 2 + 0] = t[0];
				s->st[vi * 2 + 1] = t[1];
			}

			if (isSkinned) {
				unsigned char ub[4] = {0, 0, 0, 0};
				iqm_read_ubyte(biPtr + (size_t)gi * biStride, biComp <= 4 ? biComp : 4, ub);
				memcpy(bIdx + vi * 4, ub, 4);
				float w[4] = {0, 0, 0, 0};
				if (bwFmt == IQM_UBYTE) {
					unsigned char wb[4] = {0, 0, 0, 0};
					iqm_read_ubyte(bwPtr + (size_t)gi * bwStride, bwComp <= 4 ? bwComp : 4, wb);
					for (int k = 0; k < 4; ++k)
						w[k] = wb[k] / 255.0f;
				} else {
					iqm_read_float(buf, bwFmt, bwComp <= 4 ? bwComp : 4, bwPtr + (size_t)gi * bwStride, w);
				}
				memcpy(bWgt + vi * 4, w, sizeof(w));
			}
		}

		for (unsigned int ti = 0; ti < mesh->num_triangles; ++ti) {
			const iqmTriangle_t *tri = &tris[mesh->first_triangle + ti];
			s->indices[ti * 3 + 0] = (int)tri->vertex[0] - (int)mesh->first_vertex;
			s->indices[ti * 3 + 1] = (int)tri->vertex[1] - (int)mesh->first_vertex;
			s->indices[ti * 3 + 2] = (int)tri->vertex[2] - (int)mesh->first_vertex;
		}

		/* Frame 0: emit bind-pose verts directly. */
		memcpy(s->xyz, bindPos, (size_t)mesh->num_vertexes * 3 * sizeof(float));
		memcpy(s->normal, bindNrm, (size_t)mesh->num_vertexes * 3 * sizeof(float));

		/* Persist the per-vertex bone weights on the surface so writers
		   can emit BLENDINDEXES / BLENDWEIGHTS losslessly. */
		if (isSkinned) {
			mc_surface_alloc_blend(s);
			memcpy(s->blendIndices, bIdx, (size_t)mesh->num_vertexes * 4);
			memcpy(s->blendWeights, bWgt, (size_t)mesh->num_vertexes * 4 * sizeof(float));
		}

		/* Subsequent frames: evaluate joint poses, skin every vertex. */
		if (isSkinned) {
			float (*absMats)[3][4] = (float (*)[3][4])mc_malloc(sizeof(float[3][4]) * hdr->num_joints);
			for (int f = 1; f < numFrames; ++f) {
				/* Decode joint local TRS from this frame's channel data,
				   then accumulate parent matrices. */
				const unsigned short *fd = frameData + (size_t)f * hdr->num_framechannels;
				int chan = 0;
				float frameAbs[3][4]; /* transient */
				/* We need all absolute matrices cached so children can use parent. */
				(void)frameAbs;
				for (unsigned int j = 0; j < hdr->num_joints; ++j) {
					float ch[10];
					for (int c = 0; c < 10; ++c) {
						ch[c] = poses[j].channeloffset[c];
						if (poses[j].mask & (1u << c)) {
							ch[c] += fd[chan++] * poses[j].channelscale[c];
						}
					}
					float t[3] = {ch[0], ch[1], ch[2]};
					float q[4] = {ch[3], ch[4], ch[5], ch[6]};
					float sc[3] = {ch[7], ch[8], ch[9]};
					/* Mirror into mc_model_t per-frame pose store. */
					mc_joint_pose_t *jp = &out->jointPoses[(size_t)f * out->numJoints + j];
					memcpy(jp->trans, t, sizeof(t));
					memcpy(jp->rot,   q, sizeof(q));
					memcpy(jp->scale, sc, sizeof(sc));
					float local[3][4];
					mat34_from_trs(t, q, sc, local);
					if (joints[j].parent >= 0)
						mat34_mul(absMats[joints[j].parent], local, absMats[j]);
					else
						memcpy(absMats[j], local, sizeof(local));
					/* Skin matrix = absFrame * inverse(absBind). */
					mat34_mul(absMats[j], bindAbsInv[j], frameSkin[j]);
				}

				for (unsigned int vi = 0; vi < mesh->num_vertexes; ++vi) {
					float pos[3] = {0, 0, 0};
					float nrm[3] = {0, 0, 0};
					float wsum = 0.0f;
					for (int k = 0; k < 4; ++k) {
						float w = bWgt[vi * 4 + k];
						if (w <= 0)
							continue;
						unsigned int j = bIdx[vi * 4 + k];
						if (j >= hdr->num_joints)
							continue;
						float p[3], n[3];
						mat34_xform_point(frameSkin[j], &bindPos[vi * 3], p);
						mat34_xform_dir(frameSkin[j], &bindNrm[vi * 3], n);
						pos[0] += w * p[0]; pos[1] += w * p[1]; pos[2] += w * p[2];
						nrm[0] += w * n[0]; nrm[1] += w * n[1]; nrm[2] += w * n[2];
						wsum += w;
					}
					if (wsum <= 0) {
						/* Unskinned vertex - keep bind pose. */
						memcpy(pos, &bindPos[vi * 3], sizeof(pos));
						memcpy(nrm, &bindNrm[vi * 3], sizeof(nrm));
					}
					float ln = sqrtf(nrm[0] * nrm[0] + nrm[1] * nrm[1] + nrm[2] * nrm[2]);
					if (ln > 1e-6f) { nrm[0] /= ln; nrm[1] /= ln; nrm[2] /= ln; }
					else { nrm[0] = 0; nrm[1] = 0; nrm[2] = 1; }
					const size_t base = (size_t)f * mesh->num_vertexes * 3 + (size_t)vi * 3;
					s->xyz[base + 0] = pos[0];
					s->xyz[base + 1] = pos[1];
					s->xyz[base + 2] = pos[2];
					s->normal[base + 0] = nrm[0];
					s->normal[base + 1] = nrm[1];
					s->normal[base + 2] = nrm[2];
				}
			}
			free(absMats);
		}

		free(bindPos);
		free(bindNrm);
		free(bIdx);
		free(bWgt);
	}

	/* Per-frame bounds. */
	for (int f = 0; f < numFrames; ++f) {
		mc_frame_t *fr = &out->frames[f];
		float mins[3] = {1e30f, 1e30f, 1e30f};
		float maxs[3] = {-1e30f, -1e30f, -1e30f};
		for (int i = 0; i < out->numSurfaces; ++i) {
			const mc_surface_t *s = &out->surfaces[i];
			const float *xyz = s->xyz + (size_t)f * s->numVerts * 3;
			for (int vi = 0; vi < s->numVerts; ++vi) {
				for (int k = 0; k < 3; ++k) {
					if (xyz[vi * 3 + k] < mins[k]) mins[k] = xyz[vi * 3 + k];
					if (xyz[vi * 3 + k] > maxs[k]) maxs[k] = xyz[vi * 3 + k];
				}
			}
		}
		if (mins[0] > maxs[0]) { mins[0] = mins[1] = mins[2] = 0; maxs[0] = maxs[1] = maxs[2] = 0; }
		memcpy(fr->bounds[0], mins, sizeof(mins));
		memcpy(fr->bounds[1], maxs, sizeof(maxs));
		float cx = 0.5f * (mins[0] + maxs[0]);
		float cy = 0.5f * (mins[1] + maxs[1]);
		float cz = 0.5f * (mins[2] + maxs[2]);
		float r2 = 0;
		for (int i = 0; i < out->numSurfaces; ++i) {
			const mc_surface_t *s = &out->surfaces[i];
			const float *xyz = s->xyz + (size_t)f * s->numVerts * 3;
			for (int vi = 0; vi < s->numVerts; ++vi) {
				float dx = xyz[vi * 3 + 0] - cx;
				float dy = xyz[vi * 3 + 1] - cy;
				float dz = xyz[vi * 3 + 2] - cz;
				float d2 = dx * dx + dy * dy + dz * dz;
				if (d2 > r2) r2 = d2;
			}
		}
		fr->radius = sqrtf(r2);
	}

	free(bindAbs);
	free(bindAbsInv);
	free(frameSkin);
	free(buf);

	MC_LOG("loaded iqm '%s': %d surfaces, %d frames%s\n", path, out->numSurfaces, numFrames,
		   isSkinned ? " (skinned)" : " (static)");
	return 0;
}

/* ==================================================================== */
/* IQM writer                                                           */
/* ==================================================================== */
/*
============
mc_save_iqm

Writes an IQM file from the given model.  When m->numJoints > 0 and at
least one surface has blendIndices+blendWeights, a full skeletal IQM is
emitted: joints, poses (mask 0x3FF, per-channel scale/offset computed
from the per-frame TRS extents), framedata, optional animations, plus
the POSITION/NORMAL/TEXCOORD vertex arrays and BLENDINDEXES/BLENDWEIGHTS
(UBYTE x 4, normalised to sum 255).  Otherwise a static IQM is emitted
containing only POSITION/NORMAL/TEXCOORD from frame 0 - the minimum the
renderer's IQM loader (code/renderergl1/tr_model_iqm.c) accepts.

File layout (all sections are 4-byte aligned because every value we write
is a 32-bit type):

  [iqmHeader_t                                          ]
  [text strings (NUL-joined)                            ]
  [iqmMesh_t  * numMeshes                               ]
  [iqmVertexArray_t * 3 or 5                            ]   POSITION,
                                                            NORMAL,
                                                            TEXCOORD,
                                                            (BLENDINDEXES,
                                                             BLENDWEIGHTS)
  [position floats                                      ]
  [normal floats                                        ]
  [texcoord floats                                      ]
  [blendindexes ubytes (skeletal only)                  ]
  [blendweights ubytes (skeletal only)                  ]
  [iqmTriangle_t * numTris                              ]
  [iqmJoint_t * numJoints           (skeletal only)     ]
  [iqmPose_t  * numJoints           (skeletal only)     ]
  [framedata ushorts                (skeletal only)     ]
  [iqmAnim_t  * numAnimations       (skeletal only)     ]
============
*/

/* Append helper for our growing output buffer. Returns the byte offset at
   which the data was written, then keeps the buffer 4-byte aligned. */
typedef struct {
	unsigned char *data;
	size_t size;
	size_t cap;
} iqm_buf_t;

static unsigned int iqm_append(iqm_buf_t *b, const void *src, size_t n) {
	if (b->size + n + 4 > b->cap) {
		size_t cap = b->cap ? b->cap : 4096;
		while (cap < b->size + n + 4) cap *= 2;
		b->data = (unsigned char *)mc_realloc(b->data, cap);
		b->cap = cap;
	}
	unsigned int ofs = (unsigned int)b->size;
	if (src) memcpy(b->data + b->size, src, n);
	else memset(b->data + b->size, 0, n);
	b->size += n;
	while (b->size & 3u) {
		b->data[b->size++] = 0;
	}
	return ofs;
}

/* Append a string to the text section and return its offset relative to
   the text base. The empty string is always at offset 0. */
static unsigned int iqm_text_intern(iqm_buf_t *text, const char *s) {
	if (!s) s = "";
	size_t len = strlen(s);
	/* Linear-scan for an existing copy (text sections are small). */
	for (size_t i = 0; i + len < text->size; ++i) {
		if (memcmp(text->data + i, s, len + 1) == 0) {
			return (unsigned int)i;
		}
	}
	unsigned int ofs = (unsigned int)text->size;
	if (text->size + len + 1 > text->cap) {
		size_t cap = text->cap ? text->cap : 256;
		while (cap < text->size + len + 1) cap *= 2;
		text->data = (unsigned char *)mc_realloc(text->data, cap);
		text->cap = cap;
	}
	memcpy(text->data + text->size, s, len + 1);
	text->size += len + 1;
	return ofs;
}

int mc_save_iqm(const char *path, const mc_model_t *m) {
	if (m->numSurfaces <= 0) {
		MC_ERR("iqm: cannot write a model with no surfaces\n");
		return -1;
	}

	int hasSkel = (m->numJoints > 0 && m->jointPoses != NULL);
	int frames = m->numFrames > 0 ? m->numFrames : 1;
	/* Detect blend data on at least one surface. */
	int hasBlend = 0;
	if (hasSkel) {
		for (int i = 0; i < m->numSurfaces; ++i) {
			if (m->surfaces[i].blendIndices && m->surfaces[i].blendWeights) { hasBlend = 1; break; }
		}
	}

	/* Total vertex / triangle counts across all surfaces. */
	unsigned int totalVerts = 0;
	unsigned int totalTris = 0;
	for (int i = 0; i < m->numSurfaces; ++i) {
		totalVerts += (unsigned int)m->surfaces[i].numVerts;
		totalTris += (unsigned int)m->surfaces[i].numTris;
	}

	/* Build the text section first (offsets are referenced by mesh records). */
	iqm_buf_t text = { 0 };
	iqm_text_intern(&text, ""); /* offset 0 -> empty */

	iqmMesh_t *meshes = (iqmMesh_t *)mc_calloc((size_t)m->numSurfaces, sizeof(iqmMesh_t));
	unsigned int firstVert = 0;
	unsigned int firstTri = 0;
	for (int i = 0; i < m->numSurfaces; ++i) {
		const mc_surface_t *s = &m->surfaces[i];
		const char *shader = s->shader[0] ? s->shader : (s->texture[0] ? s->texture : "");
		meshes[i].name = iqm_text_intern(&text, s->name[0] ? s->name : "");
		meshes[i].material = iqm_text_intern(&text, shader);
		meshes[i].first_vertex = firstVert;
		meshes[i].num_vertexes = (unsigned int)s->numVerts;
		meshes[i].first_triangle = firstTri;
		meshes[i].num_triangles = (unsigned int)s->numTris;
		firstVert += (unsigned int)s->numVerts;
		firstTri += (unsigned int)s->numTris;
	}

	/* Joints. */
	iqmJoint_t *iqJoints = NULL;
	if (hasSkel) {
		iqJoints = (iqmJoint_t *)mc_calloc((size_t)m->numJoints, sizeof(iqmJoint_t));
		for (int j = 0; j < m->numJoints; ++j) {
			const mc_joint_t *mj = &m->joints[j];
			iqJoints[j].name = iqm_text_intern(&text, mj->name);
			iqJoints[j].parent = mj->parent;
			memcpy(iqJoints[j].translate, mj->bindTrans, sizeof(iqJoints[j].translate));
			memcpy(iqJoints[j].rotate,    mj->bindRot,   sizeof(iqJoints[j].rotate));
			memcpy(iqJoints[j].scale,     mj->bindScale, sizeof(iqJoints[j].scale));
		}
	}

	/* Pre-intern anim names so the text section is final before we emit it. */
	unsigned int *animNameOfs = NULL;
	if (hasSkel && m->numAnimations > 0) {
		animNameOfs = (unsigned int *)mc_calloc((size_t)m->numAnimations, sizeof(unsigned int));
		for (int a = 0; a < m->numAnimations; ++a)
			animNameOfs[a] = iqm_text_intern(&text, m->animations[a].name);
	}

	/* Poses: encode every joint with all 10 channels active (mask = 0x3FF)
	   and channelscale = 1, channeloffset = 0.  Each frame channel value
	   is then `value = framedata[i]` directly, simplifying the writer. */
	iqmPose_t *iqPoses = NULL;
	unsigned int numFrameChannels = 0;
	if (hasSkel) {
		iqPoses = (iqmPose_t *)mc_calloc((size_t)m->numJoints, sizeof(iqmPose_t));
		for (int j = 0; j < m->numJoints; ++j) {
			iqPoses[j].parent = m->joints[j].parent;
			iqPoses[j].mask = 0x3FFu;
			for (int c = 0; c < 10; ++c) {
				iqPoses[j].channeloffset[c] = 0.0f;
				iqPoses[j].channelscale[c] = 1.0f;
			}
		}
		numFrameChannels = (unsigned int)m->numJoints * 10u;
	}

	/* Stream the file body into a single buffer with running offsets. */
	iqm_buf_t out = { 0 };
	iqmHeader_t hdr;
	memset(&hdr, 0, sizeof(hdr));
	memcpy(hdr.magic, IQM_MAGIC, 16);
	hdr.version = IQM_VERSION;

	iqm_append(&out, &hdr, sizeof(hdr));

	/* Text. */
	hdr.ofs_text = iqm_append(&out, text.data, text.size);
	hdr.num_text = (unsigned int)text.size;

	/* Meshes. */
	hdr.ofs_meshes = iqm_append(&out, meshes, sizeof(iqmMesh_t) * (size_t)m->numSurfaces);
	hdr.num_meshes = (unsigned int)m->numSurfaces;

	/* Vertex array descriptors get patched after data is appended. */
	int numVA = 3 + (hasBlend ? 2 : 0);
	unsigned int vaOfs = iqm_append(&out, NULL, sizeof(iqmVertexArray_t) * (size_t)numVA);
	hdr.ofs_vertexarrays = vaOfs;
	hdr.num_vertexarrays = (unsigned int)numVA;
	hdr.num_vertexes = totalVerts;

	/* POSITION. */
	unsigned int posOfs = (unsigned int)out.size;
	for (int i = 0; i < m->numSurfaces; ++i) {
		const mc_surface_t *s = &m->surfaces[i];
		iqm_append(&out, s->xyz, (size_t)s->numVerts * 3 * sizeof(float));
	}
	/* NORMAL. */
	unsigned int normOfs = (unsigned int)out.size;
	for (int i = 0; i < m->numSurfaces; ++i) {
		const mc_surface_t *s = &m->surfaces[i];
		iqm_append(&out, s->normal, (size_t)s->numVerts * 3 * sizeof(float));
	}
	/* TEXCOORD. */
	unsigned int uvOfs = (unsigned int)out.size;
	for (int i = 0; i < m->numSurfaces; ++i) {
		const mc_surface_t *s = &m->surfaces[i];
		iqm_append(&out, s->st, (size_t)s->numVerts * 2 * sizeof(float));
	}
	/* BLENDINDEXES (UBYTE x 4) + BLENDWEIGHTS (UBYTE x 4 normalised). */
	unsigned int biOfs = 0, bwOfs = 0;
	if (hasBlend) {
		biOfs = (unsigned int)out.size;
		for (int i = 0; i < m->numSurfaces; ++i) {
			const mc_surface_t *s = &m->surfaces[i];
			if (s->blendIndices)
				iqm_append(&out, s->blendIndices, (size_t)s->numVerts * 4);
			else {
				/* Surface lacks blend data - fill with joint 0 / weight 0. */
				unsigned char *zero = (unsigned char *)mc_calloc((size_t)s->numVerts * 4, 1);
				iqm_append(&out, zero, (size_t)s->numVerts * 4);
				free(zero);
			}
		}
		bwOfs = (unsigned int)out.size;
		for (int i = 0; i < m->numSurfaces; ++i) {
			const mc_surface_t *s = &m->surfaces[i];
			unsigned char *wb = (unsigned char *)mc_calloc((size_t)s->numVerts * 4, 1);
			if (s->blendWeights) {
				for (int v = 0; v < s->numVerts; ++v) {
					/* Renormalise to 0..255 sum 255 to satisfy the IQM spec. */
					float sum = 0;
					for (int k = 0; k < 4; ++k) sum += s->blendWeights[v * 4 + k];
					float inv = sum > 1e-6f ? 255.0f / sum : 0.0f;
					int total = 0;
					int largest = 0; int largestVal = -1;
					for (int k = 0; k < 4; ++k) {
						int b = (int)(s->blendWeights[v * 4 + k] * inv + 0.5f);
						if (b < 0) b = 0;
						if (b > 255) b = 255;
						wb[v * 4 + k] = (unsigned char)b;
						total += b;
						if (b > largestVal) { largestVal = b; largest = k; }
					}
					/* Drift correction so weights sum to exactly 255. */
					int diff = (sum > 1e-6f ? 255 : 0) - total;
					int corrected = (int)wb[v * 4 + largest] + diff;
					if (corrected < 0) corrected = 0;
					if (corrected > 255) corrected = 255;
					wb[v * 4 + largest] = (unsigned char)corrected;
				}
			}
			iqm_append(&out, wb, (size_t)s->numVerts * 4);
			free(wb);
		}
	}

	/* Patch the vertex array descriptors. */
	iqmVertexArray_t vas[5];
	memset(vas, 0, sizeof(vas));
	vas[0].type = IQM_POSITION; vas[0].format = IQM_FLOAT; vas[0].size = 3; vas[0].offset = posOfs;
	vas[1].type = IQM_NORMAL;   vas[1].format = IQM_FLOAT; vas[1].size = 3; vas[1].offset = normOfs;
	vas[2].type = IQM_TEXCOORD; vas[2].format = IQM_FLOAT; vas[2].size = 2; vas[2].offset = uvOfs;
	if (hasBlend) {
		vas[3].type = IQM_BLENDINDEXES; vas[3].format = IQM_UBYTE; vas[3].size = 4; vas[3].offset = biOfs;
		vas[4].type = IQM_BLENDWEIGHTS; vas[4].format = IQM_UBYTE; vas[4].size = 4; vas[4].offset = bwOfs;
	}
	memcpy(out.data + vaOfs, vas, sizeof(iqmVertexArray_t) * (size_t)numVA);

	/* Triangles. */
	hdr.ofs_triangles = (unsigned int)out.size;
	hdr.num_triangles = totalTris;
	for (int i = 0; i < m->numSurfaces; ++i) {
		const mc_surface_t *s = &m->surfaces[i];
		unsigned int base = meshes[i].first_vertex;
		for (int t = 0; t < s->numTris; ++t) {
			iqmTriangle_t tri;
			tri.vertex[0] = base + (unsigned int)s->indices[t * 3 + 0];
			tri.vertex[1] = base + (unsigned int)s->indices[t * 3 + 1];
			tri.vertex[2] = base + (unsigned int)s->indices[t * 3 + 2];
			iqm_append(&out, &tri, sizeof(tri));
		}
	}

	/* Joints / poses / frames / anims. */
	if (hasSkel) {
		hdr.ofs_joints = iqm_append(&out, iqJoints, sizeof(iqmJoint_t) * (size_t)m->numJoints);
		hdr.num_joints = (unsigned int)m->numJoints;
		hdr.ofs_poses = iqm_append(&out, iqPoses, sizeof(iqmPose_t) * (size_t)m->numJoints);
		hdr.num_poses = (unsigned int)m->numJoints;
		hdr.num_framechannels = numFrameChannels;
		hdr.num_frames = (unsigned int)frames;
		/* Frame data: frames * numJoints * 10 ushorts.
		   Since channelscale==1 and channeloffset==0, the encoded value
		   must equal the actual TRS value.  TRS values can be negative
		   or > 65535; to support that range we'd need real
		   channeloffset/channelscale per-channel.  Compute them now. */
		float *minCh = (float *)mc_malloc(sizeof(float) * 10 * (size_t)m->numJoints);
		float *maxCh = (float *)mc_malloc(sizeof(float) * 10 * (size_t)m->numJoints);
		for (int j = 0; j < m->numJoints; ++j) {
			for (int c = 0; c < 10; ++c) { minCh[j * 10 + c] = 1e30f; maxCh[j * 10 + c] = -1e30f; }
		}
		for (int f = 0; f < frames; ++f) {
			for (int j = 0; j < m->numJoints; ++j) {
				const mc_joint_pose_t *p = &m->jointPoses[(size_t)f * m->numJoints + j];
				float ch[10] = { p->trans[0], p->trans[1], p->trans[2],
								 p->rot[0],   p->rot[1],   p->rot[2], p->rot[3],
								 p->scale[0], p->scale[1], p->scale[2] };
				for (int c = 0; c < 10; ++c) {
					if (ch[c] < minCh[j * 10 + c]) minCh[j * 10 + c] = ch[c];
					if (ch[c] > maxCh[j * 10 + c]) maxCh[j * 10 + c] = ch[c];
				}
			}
		}
		/* Patch the pose records with proper offsets/scales now that we
		   know the value range.  We always emit 65535 ticks per channel. */
		for (int j = 0; j < m->numJoints; ++j) {
			for (int c = 0; c < 10; ++c) {
				float lo = minCh[j * 10 + c], hi = maxCh[j * 10 + c];
				float range = hi - lo;
				iqPoses[j].channeloffset[c] = lo;
				iqPoses[j].channelscale[c] = range > 1e-12f ? range / 65535.0f : 0.0f;
			}
		}
		/* Re-write the patched poses. */
		memcpy(out.data + hdr.ofs_poses, iqPoses, sizeof(iqmPose_t) * (size_t)m->numJoints);

		hdr.ofs_frames = (unsigned int)out.size;
		/* Build the entire framedata block in one buffer and append it
		   in a single call, because iqm_append pads every call to a
		   4-byte boundary -- appending one ushort at a time would
		   waste 2 bytes per channel and corrupt the layout. */
		size_t framesBytes = (size_t)frames * (size_t)m->numJoints * 10u * sizeof(unsigned short);
		unsigned short *frameBlock = (unsigned short *)mc_malloc(framesBytes);
		size_t fi = 0;
		for (int f = 0; f < frames; ++f) {
			for (int j = 0; j < m->numJoints; ++j) {
				const mc_joint_pose_t *p = &m->jointPoses[(size_t)f * m->numJoints + j];
				float ch[10] = { p->trans[0], p->trans[1], p->trans[2],
								 p->rot[0],   p->rot[1],   p->rot[2], p->rot[3],
								 p->scale[0], p->scale[1], p->scale[2] };
				for (int c = 0; c < 10; ++c) {
					unsigned short v = 0;
					float scale = iqPoses[j].channelscale[c];
					if (scale > 0) {
						float vf = (ch[c] - iqPoses[j].channeloffset[c]) / scale;
						if (vf < 0) vf = 0;
						if (vf > 65535) vf = 65535;
						v = (unsigned short)(vf + 0.5f);
					}
					frameBlock[fi++] = v;
				}
			}
		}
		iqm_append(&out, frameBlock, framesBytes);
		free(frameBlock);
		free(minCh);
		free(maxCh);

		/* Anims. */
		if (m->numAnimations > 0) {
			iqmAnim_t *iqAnims = (iqmAnim_t *)mc_calloc((size_t)m->numAnimations, sizeof(iqmAnim_t));
			for (int a = 0; a < m->numAnimations; ++a) {
				iqAnims[a].name = animNameOfs ? animNameOfs[a] : 0;
				iqAnims[a].first_frame = (unsigned int)m->animations[a].firstFrame;
				iqAnims[a].num_frames = (unsigned int)m->animations[a].numFrames;
				iqAnims[a].framerate = m->animations[a].fps > 0 ? m->animations[a].fps : 15.0f;
				iqAnims[a].flags = m->animations[a].loopFrames > 0 ? IQM_LOOP : 0;
			}
			hdr.ofs_anims = iqm_append(&out, iqAnims, sizeof(iqmAnim_t) * (size_t)m->numAnimations);
			hdr.num_anims = (unsigned int)m->numAnimations;
			free(iqAnims);
		}

		/* Refresh text section in case anim names extended it. */
		/* (We embedded text earlier but joint/anim text was interned into
		   the same buffer; if anims extended it after we wrote it, those
		   string offsets are stale.  Avoid that by re-emitting if needed.) */
	}

	/* Patch the header in place and finalise the filesize. */
	hdr.filesize = (unsigned int)out.size;
	memcpy(out.data, &hdr, sizeof(hdr));

	int rc = mc_write_file(path, out.data, out.size);

	free(meshes);
	free(iqJoints);
	free(iqPoses);
	free(animNameOfs);
	free(text.data);
	free(out.data);
	return rc;
}

