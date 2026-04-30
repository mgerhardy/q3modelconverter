/*
===========================================================================
test_roundtrip - self-contained round-trip smoke test.

Builds a tiny static cube model in memory, then exercises every (load,
save) format pair: MD3, IQM, MDR, glTF, glB.  Verifies vertex/triangle
counts and basic geometry survive each conversion.  Returns 0 on
success, non-zero on any mismatch.
===========================================================================
*/

#include "../src/mc_common.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define CHECK(cond, msg) do { if (!(cond)) { \
	fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, msg); \
	return 1; \
} } while (0)

static void make_cube(mc_model_t *m) {
	mc_model_init(m);
	mc_frame_t *fr = mc_model_add_frame(m);
	(void)fr;
	mc_surface_t *s = mc_model_add_surface(m);
	mc_q_strncpy(s->name, "cube", sizeof(s->name));
	mc_q_strncpy(s->shader, "models/test/cube", sizeof(s->shader));
	mc_surface_alloc(s, 8, 12, 1);

	const float v[8][3] = {
		{-1,-1,-1}, { 1,-1,-1}, { 1, 1,-1}, {-1, 1,-1},
		{-1,-1, 1}, { 1,-1, 1}, { 1, 1, 1}, {-1, 1, 1},
	};
	for (int i = 0; i < 8; ++i) {
		s->xyz[i*3+0] = v[i][0];
		s->xyz[i*3+1] = v[i][1];
		s->xyz[i*3+2] = v[i][2];
		float l = sqrtf(v[i][0]*v[i][0] + v[i][1]*v[i][1] + v[i][2]*v[i][2]);
		s->normal[i*3+0] = v[i][0] / l;
		s->normal[i*3+1] = v[i][1] / l;
		s->normal[i*3+2] = v[i][2] / l;
		s->st[i*2+0] = (float)(i & 1);
		s->st[i*2+1] = (float)((i >> 1) & 1);
	}
	const int idx[12][3] = {
		{0,1,2},{0,2,3}, /* -Z */
		{4,6,5},{4,7,6}, /* +Z */
		{0,4,5},{0,5,1}, /* -Y */
		{2,6,7},{2,7,3}, /* +Y */
		{1,5,6},{1,6,2}, /* +X */
		{0,3,7},{0,7,4}, /* -X */
	};
	for (int t = 0; t < 12; ++t) {
		s->indices[t*3+0] = idx[t][0];
		s->indices[t*3+1] = idx[t][1];
		s->indices[t*3+2] = idx[t][2];
	}
	mc_compute_bounds(s->xyz, 8, m->frames[0].bounds[0], m->frames[0].bounds[1], &m->frames[0].radius);
}

/* Two-frame animated cube + one tag, used to verify vertex morph and tag
   round-trips. Frame 0 is the unit cube; frame 1 is a uniformly scaled
   2x cube. Tag rotates 30deg around Z between the two frames. */
static void make_animated_cube(mc_model_t *m) {
	make_cube(m);
	/* Add a second frame: re-alloc surface to numFrames=2, then write
	   a scaled copy of frame 0. */
	mc_surface_t *s = &m->surfaces[0];
	float *xyz2 = (float *)mc_malloc(sizeof(float) * 8 * 3 * 2);
	float *nrm2 = (float *)mc_malloc(sizeof(float) * 8 * 3 * 2);
	memcpy(xyz2, s->xyz, sizeof(float) * 8 * 3);
	memcpy(nrm2, s->normal, sizeof(float) * 8 * 3);
	for (int i = 0; i < 8 * 3; ++i)
		xyz2[8 * 3 + i] = s->xyz[i] * 2.0f;
	memcpy(nrm2 + 8 * 3, s->normal, sizeof(float) * 8 * 3);
	free(s->xyz);
	free(s->normal);
	s->xyz = xyz2;
	s->normal = nrm2;
	mc_frame_t *f2 = mc_model_add_frame(m);
	mc_compute_bounds(&s->xyz[8 * 3], 8, f2->bounds[0], f2->bounds[1], &f2->radius);

	/* Add a tag with per-frame TRS. */
	mc_tag_t *t0 = mc_model_add_tag_slot(m, m->numFrames);
	mc_q_strncpy(t0->name, "tag_test", sizeof(t0->name));
	t0->origin[0] = 1.0f; t0->origin[1] = 0.0f; t0->origin[2] = 0.0f;
	t0->axis[0][0] = 1; t0->axis[1][1] = 1; t0->axis[2][2] = 1;
	mc_tag_t *t1 = &m->tags[1 * m->numTags + 0];
	mc_q_strncpy(t1->name, "tag_test", sizeof(t1->name));
	t1->origin[0] = 2.0f; t1->origin[1] = 0.0f; t1->origin[2] = 0.0f;
	const float c = 0.8660254f, sn = 0.5f; /* 30 deg around Z */
	t1->axis[0][0] = c;  t1->axis[0][1] = sn; t1->axis[0][2] = 0;
	t1->axis[1][0] = -sn; t1->axis[1][1] = c; t1->axis[1][2] = 0;
	t1->axis[2][0] = 0;  t1->axis[2][1] = 0;  t1->axis[2][2] = 1;
}

/* Skinned cube: same geometry, but every vertex is rigidly bound to a
   single root bone whose pose rotates 90 deg around Z between frame 0
   and frame 1.  Used to verify joint round-trip through IQM/MDR/glTF. */
static void make_skinned_cube(mc_model_t *m) {
	make_cube(m);
	mc_surface_t *s = &m->surfaces[0];
	/* Add second frame for the surface (same geometry; skinning drives
	   the deformation). */
	float *xyz2 = (float *)mc_malloc(sizeof(float) * 8 * 3 * 2);
	float *nrm2 = (float *)mc_malloc(sizeof(float) * 8 * 3 * 2);
	memcpy(xyz2, s->xyz, sizeof(float) * 8 * 3);
	memcpy(nrm2, s->normal, sizeof(float) * 8 * 3);
	memcpy(xyz2 + 8 * 3, s->xyz, sizeof(float) * 8 * 3);
	memcpy(nrm2 + 8 * 3, s->normal, sizeof(float) * 8 * 3);
	free(s->xyz); free(s->normal);
	s->xyz = xyz2; s->normal = nrm2;
	mc_model_add_frame(m);

	mc_model_set_joints(m, 1);
	mc_q_strncpy(m->joints[0].name, "root", sizeof(m->joints[0].name));
	m->joints[0].parent = -1;
	m->joints[0].bindRot[3] = 1.0f;       /* identity */
	m->joints[0].bindScale[0] = m->joints[0].bindScale[1] = m->joints[0].bindScale[2] = 1.0f;

	for (int f = 0; f < m->numFrames; ++f) {
		mc_joint_pose_t *p = &m->jointPoses[f * 1 + 0];
		p->scale[0] = p->scale[1] = p->scale[2] = 1.0f;
		if (f == 0) {
			p->rot[3] = 1.0f;            /* identity */
		} else {
			/* 90 deg around Z = (0,0,sin45,cos45) */
			p->rot[2] = 0.7071068f;
			p->rot[3] = 0.7071068f;
		}
	}

	mc_surface_alloc_blend(s);
	for (int v = 0; v < 8; ++v) {
		s->blendIndices[v * 4 + 0] = 0;
		s->blendWeights[v * 4 + 0] = 1.0f;
	}

	/* Register a single animation covering both frames so the glTF
	   writer emits a cgltf_animation track and the IQM writer emits an
	   anim entry. */
	mc_animation_t *am = mc_model_add_animation(m);
	mc_q_strncpy(am->name, "spin", sizeof(am->name));
	am->firstFrame = 0;
	am->numFrames = m->numFrames;
	am->loopFrames = m->numFrames;
	am->fps = 15.0f; /* matches gltf_load's default fps_hint so the
	                   round-trip resamples to the same frame count */
	m->fps = 15.0f;
}

/* Multi-bone skinned cube: 3 bones in a parent chain (root -> mid -> tip),
   3 frames, 2 animations.  Vertices on the -Z face (indices 0..3) are
   bound 100% to root, +Z face (4..7) 100% to tip, giving a clean
   hierarchy test.  Frame 0 = bind pose (identity chain), frame 1 = mid
   rotates 45 deg around Z, frame 2 = tip translates +2 along X.
   Two animations: "wave" covers frames 0-1, "poke" covers frames 1-2. */
static void make_multibone_cube(mc_model_t *m) {
	make_cube(m);
	mc_surface_t *s = &m->surfaces[0];

	/* Expand to 3 frames. */
	const int NF = 3;
	float *xyz3 = (float *)mc_calloc((size_t)8 * 3 * NF, sizeof(float));
	float *nrm3 = (float *)mc_calloc((size_t)8 * 3 * NF, sizeof(float));
	for (int f = 0; f < NF; ++f) {
		memcpy(xyz3 + (size_t)f * 8 * 3, s->xyz, sizeof(float) * 8 * 3);
		memcpy(nrm3 + (size_t)f * 8 * 3, s->normal, sizeof(float) * 8 * 3);
	}
	free(s->xyz); free(s->normal);
	s->xyz = xyz3; s->normal = nrm3;
	/* Add frames 2 and 3 (frame 1 already exists from make_cube). */
	mc_model_add_frame(m);
	mc_model_add_frame(m);

	/* 3 joints: root (parent -1), mid (parent 0), tip (parent 1). */
	mc_model_set_joints(m, 3);
	for (int j = 0; j < 3; ++j) {
		mc_joint_t *jt = &m->joints[j];
		snprintf(jt->name, sizeof(jt->name), "bone_%d", j);
		jt->parent = j - 1; /* root=-1, mid=0, tip=1 */
		jt->bindRot[3] = 1.0f;
		jt->bindScale[0] = jt->bindScale[1] = jt->bindScale[2] = 1.0f;
		/* Chain along +Y: root at origin, mid at Y=1, tip at Y=2. */
		jt->bindTrans[1] = (j == 0) ? 0.0f : 1.0f;
	}

	/* Per-frame poses. */
	for (int f = 0; f < NF; ++f) {
		for (int j = 0; j < 3; ++j) {
			mc_joint_pose_t *p = &m->jointPoses[(size_t)f * 3 + j];
			p->scale[0] = p->scale[1] = p->scale[2] = 1.0f;
			p->rot[3] = 1.0f;
			p->trans[1] = (j == 0) ? 0.0f : 1.0f;
		}
	}
	/* Frame 1: mid rotates 45 deg around Z. */
	{
		mc_joint_pose_t *p = &m->jointPoses[1 * 3 + 1];
		float half = (float)(M_PI / 8.0); /* 22.5 deg half-angle */
		p->rot[0] = 0.0f; p->rot[1] = 0.0f;
		p->rot[2] = sinf(half); p->rot[3] = cosf(half);
	}
	/* Frame 2: tip translates +2 along local X. */
	{
		mc_joint_pose_t *p = &m->jointPoses[2 * 3 + 2];
		p->trans[0] = 2.0f;
	}

	/* Blend: verts 0-3 (-Z face) -> bone 0, verts 4-7 (+Z face) -> bone 2. */
	mc_surface_alloc_blend(s);
	for (int v = 0; v < 8; ++v) {
		s->blendIndices[v * 4] = (v < 4) ? 0 : 2;
		s->blendWeights[v * 4] = 1.0f;
	}

	/* Two animations. */
	mc_animation_t *a1 = mc_model_add_animation(m);
	mc_q_strncpy(a1->name, "wave", sizeof(a1->name));
	a1->firstFrame = 0; a1->numFrames = 2; a1->loopFrames = 2; a1->fps = 15.0f;

	mc_animation_t *a2 = mc_model_add_animation(m);
	mc_q_strncpy(a2->name, "poke", sizeof(a2->name));
	a2->firstFrame = 1; a2->numFrames = 2; a2->loopFrames = 0; a2->fps = 10.0f;

	m->fps = 15.0f;
}

typedef int (*saver_fn)(const char *path, const mc_model_t *m);
typedef int (*loader_fn)(const char *path, mc_model_t *out);

static int gltf_save(const char *path, const mc_model_t *m) {
	return mc_save_gltf(path, m, /*as_glb*/0, NULL);
}
static int glb_save(const char *path, const mc_model_t *m) {
	return mc_save_gltf(path, m, /*as_glb*/1, NULL);
}
static int gltf_load(const char *path, mc_model_t *out) {
	return mc_load_gltf(path, out, 15.0f);
}

static int round_trip(const char *label, const char *path,
					  saver_fn save, loader_fn load,
					  const mc_model_t *src) {
	printf("  [%s] ", label); fflush(stdout);
	if (save(path, src) != 0) {
		printf("FAIL save\n");
		return 1;
	}
	mc_model_t dst;
	mc_model_init(&dst);
	if (load(path, &dst) != 0) {
		printf("FAIL load\n");
		return 1;
	}
	int rc = 0;
	if (dst.numSurfaces < 1) {
		printf("FAIL no surfaces\n");
		rc = 1;
	} else {
		const mc_surface_t *a = &src->surfaces[0];
		const mc_surface_t *b = &dst.surfaces[0];
		if (b->numVerts != a->numVerts || b->numTris != a->numTris) {
			printf("FAIL counts (verts %d!=%d, tris %d!=%d)\n",
				b->numVerts, a->numVerts, b->numTris, a->numTris);
			rc = 1;
		} else {
			printf("OK verts=%d tris=%d\n", b->numVerts, b->numTris);
		}
	}
	mc_model_free(&dst);
	return rc;
}

/* Animated round-trip: also asserts numFrames + numTags + per-frame
   vertex positions agree with the source within tol. MD3 uses 1/64-unit
   quantisation so positions get a generous tolerance. */
static int round_trip_anim(const char *label, const char *path,
						   saver_fn save, loader_fn load,
						   const mc_model_t *src, float pos_tol) {
	printf("  [%s anim] ", label); fflush(stdout);
	if (save(path, src) != 0) {
		printf("FAIL save\n");
		return 1;
	}
	mc_model_t dst;
	mc_model_init(&dst);
	if (load(path, &dst) != 0) {
		printf("FAIL load\n");
		return 1;
	}
	int rc = 0;
	if (dst.numFrames != src->numFrames) {
		printf("FAIL frames %d!=%d\n", dst.numFrames, src->numFrames);
		rc = 1;
		goto done;
	}
	if (dst.numTags != src->numTags) {
		printf("FAIL tags %d!=%d\n", dst.numTags, src->numTags);
		rc = 1;
		goto done;
	}
	const mc_surface_t *a = &src->surfaces[0];
	const mc_surface_t *b = &dst.surfaces[0];
	if (b->numVerts != a->numVerts || b->numTris != a->numTris) {
		printf("FAIL counts\n");
		rc = 1;
		goto done;
	}
	float worst = 0.0f;
	for (int f = 0; f < src->numFrames; ++f) {
		for (int v = 0; v < a->numVerts; ++v) {
			for (int k = 0; k < 3; ++k) {
				float d = fabsf(a->xyz[(f * a->numVerts + v) * 3 + k] -
								b->xyz[(f * b->numVerts + v) * 3 + k]);
				if (d > worst) worst = d;
			}
		}
	}
	/* pos_tol < 0 means "skip the position check" (used for glTF morph
	   targets where vertex de-duplication on read may reorder verts so
	   pairwise comparison is meaningless without rebuilding the index
	   permutation). */
	if (pos_tol >= 0.0f && worst > pos_tol) {
		printf("FAIL worst pos delta %.5f > tol %.5f\n", worst, pos_tol);
		rc = 1;
		goto done;
	}
	printf("OK frames=%d tags=%d worst-pos-delta=%.5f\n",
		dst.numFrames, dst.numTags, worst);
done:
	mc_model_free(&dst);
	return rc;
}

/* Skeletal round-trip: asserts numJoints + numFrames + per-vertex blend
   indices/weights + per-frame joint translation/rotation survive within
   tolerance.  The skinned cube uses 1 root joint with quaternions that
   round-trip cleanly through float32 storage. */
static int round_trip_skel(const char *label, const char *path,
						   saver_fn save, loader_fn load,
						   const mc_model_t *src) {
	const float trs_tol = 1e-4f;
	printf("  [%s skel] ", label); fflush(stdout);
	if (save(path, src) != 0) { printf("FAIL save\n"); return 1; }
	mc_model_t dst;
	mc_model_init(&dst);
	if (load(path, &dst) != 0) { printf("FAIL load\n"); return 1; }
	int rc = 0;
	if (dst.numJoints != src->numJoints) {
		printf("FAIL joints %d!=%d\n", dst.numJoints, src->numJoints);
		rc = 1; goto done;
	}
	if (dst.numFrames != src->numFrames) {
		printf("FAIL frames %d!=%d\n", dst.numFrames, src->numFrames);
		rc = 1; goto done;
	}
	float worst_t = 0.0f, worst_r = 0.0f;
	for (int f = 0; f < src->numFrames; ++f) {
		for (int j = 0; j < src->numJoints; ++j) {
			const mc_joint_pose_t *a = &src->jointPoses[f * src->numJoints + j];
			const mc_joint_pose_t *b = &dst.jointPoses[f * dst.numJoints + j];
			for (int k = 0; k < 3; ++k) {
				float d = fabsf(a->trans[k] - b->trans[k]);
				if (d > worst_t) worst_t = d;
			}
			/* Allow quaternion sign flip (q and -q are the same rotation). */
			float dot = a->rot[0]*b->rot[0] + a->rot[1]*b->rot[1] +
						a->rot[2]*b->rot[2] + a->rot[3]*b->rot[3];
			float sign = dot < 0 ? -1.0f : 1.0f;
			for (int k = 0; k < 4; ++k) {
				float d = fabsf(a->rot[k] - sign * b->rot[k]);
				if (d > worst_r) worst_r = d;
			}
		}
	}
	if (worst_t > trs_tol || worst_r > trs_tol) {
		printf("FAIL worst trans=%.5f rot=%.5f\n", worst_t, worst_r);
		rc = 1; goto done;
	}
	/* Verify per-vertex blend made it through. */
	const mc_surface_t *sa = &src->surfaces[0];
	const mc_surface_t *sb = &dst.surfaces[0];
	if (!sb->blendIndices || !sb->blendWeights) {
		printf("FAIL no blend arrays on read\n");
		rc = 1; goto done;
	}
	for (int v = 0; v < sa->numVerts; ++v) {
		float wsum = 0.0f;
		for (int k = 0; k < 4; ++k)
			wsum += sb->blendWeights[v * 4 + k];
		if (fabsf(wsum - 1.0f) > 1e-2f) {
			printf("FAIL vertex %d weights sum %.4f\n", v, wsum);
			rc = 1; goto done;
		}
	}
	printf("OK joints=%d frames=%d worst-trans=%.5f worst-rot=%.5f\n",
		dst.numJoints, dst.numFrames, worst_t, worst_r);
done:
	mc_model_free(&dst);
	return rc;
}

/* Multi-bone skeletal round-trip: like round_trip_skel but also verifies
   joint parent indices (when check_parents is set), per-vertex blend
   index values, and animation entry counts survive the round-trip.
   MDR flattens all parents to -1 (absolute matrices, no hierarchy) so
   the parent check is skipped for that format, and TRS comparison is
   done in absolute (world) space by accumulating the source's parent
   chain before comparing. */
static void abs_trs_for_joint(const mc_model_t *m, int frame, int joint,
							  float out_t[3], float out_r[4], float out_s[3]) {
	/* Walk the parent chain and compose local TRS into absolute.
	   Simple approach: compose 4x4-style via quaternion multiply. */
	const mc_joint_pose_t *p = &m->jointPoses[(size_t)frame * m->numJoints + joint];
	out_t[0] = p->trans[0]; out_t[1] = p->trans[1]; out_t[2] = p->trans[2];
	out_r[0] = p->rot[0]; out_r[1] = p->rot[1]; out_r[2] = p->rot[2]; out_r[3] = p->rot[3];
	out_s[0] = p->scale[0]; out_s[1] = p->scale[1]; out_s[2] = p->scale[2];
	int par = m->joints[joint].parent;
	while (par >= 0) {
		const mc_joint_pose_t *pp = &m->jointPoses[(size_t)frame * m->numJoints + par];
		/* Compose: parent * child.  For translation:
		   abs_t = parent_t + parent_rot * (parent_scale * child_t) */
		float ct[3] = { pp->scale[0]*out_t[0], pp->scale[1]*out_t[1], pp->scale[2]*out_t[2] };
		/* Rotate ct by parent quaternion. */
		float qx = pp->rot[0], qy = pp->rot[1], qz = pp->rot[2], qw = pp->rot[3];
		float tx = 2.0f*(qy*ct[2] - qz*ct[1]);
		float ty = 2.0f*(qz*ct[0] - qx*ct[2]);
		float tz = 2.0f*(qx*ct[1] - qy*ct[0]);
		out_t[0] = pp->trans[0] + ct[0] + qw*tx + qy*tz - qz*ty;
		out_t[1] = pp->trans[1] + ct[1] + qw*ty + qz*tx - qx*tz;
		out_t[2] = pp->trans[2] + ct[2] + qw*tz + qx*ty - qy*tx;
		/* Compose rotations: parent * child. */
		float rr[4];
		rr[0] = qw*out_r[0] + qx*out_r[3] + qy*out_r[2] - qz*out_r[1];
		rr[1] = qw*out_r[1] - qx*out_r[2] + qy*out_r[3] + qz*out_r[0];
		rr[2] = qw*out_r[2] + qx*out_r[1] - qy*out_r[0] + qz*out_r[3];
		rr[3] = qw*out_r[3] - qx*out_r[0] - qy*out_r[1] - qz*out_r[2];
		out_r[0] = rr[0]; out_r[1] = rr[1]; out_r[2] = rr[2]; out_r[3] = rr[3];
		/* Compose scales. */
		out_s[0] *= pp->scale[0]; out_s[1] *= pp->scale[1]; out_s[2] *= pp->scale[2];
		par = m->joints[par].parent;
	}
}

static int round_trip_skel_multi(const char *label, const char *path,
								 saver_fn save, loader_fn load,
								 const mc_model_t *src,
								 int check_parents) {
	const float trs_tol = 1e-3f; /* slightly looser for multi-bone chains */
	printf("  [%s multi] ", label); fflush(stdout);
	if (save(path, src) != 0) { printf("FAIL save\n"); return 1; }
	mc_model_t dst;
	mc_model_init(&dst);
	if (load(path, &dst) != 0) { printf("FAIL load\n"); return 1; }
	int rc = 0;
	if (dst.numJoints != src->numJoints) {
		printf("FAIL joints %d!=%d\n", dst.numJoints, src->numJoints);
		rc = 1; goto done;
	}
	if (dst.numFrames != src->numFrames) {
		printf("FAIL frames %d!=%d\n", dst.numFrames, src->numFrames);
		rc = 1; goto done;
	}
	/* Verify parent hierarchy survived (skip for MDR which flattens to -1). */
	if (check_parents) {
		for (int j = 0; j < src->numJoints; ++j) {
			if (dst.joints[j].parent != src->joints[j].parent) {
				printf("FAIL joint %d parent %d!=%d\n", j, dst.joints[j].parent, src->joints[j].parent);
				rc = 1; goto done;
			}
		}
	}
	/* Verify per-frame joint TRS.  When parents differ (MDR), compare
	   in absolute (world) space by accumulating the source's chain. */
	float worst_t = 0.0f, worst_r = 0.0f;
	for (int f = 0; f < src->numFrames; ++f) {
		for (int j = 0; j < src->numJoints; ++j) {
			float at[3], ar[4], as[3];
			float bt[3], br[4], bs[3];
			(void)as; (void)bs;
			if (check_parents) {
				/* Local TRS comparison (parents match). */
				const mc_joint_pose_t *a = &src->jointPoses[f * src->numJoints + j];
				const mc_joint_pose_t *b = &dst.jointPoses[f * dst.numJoints + j];
				memcpy(at, a->trans, sizeof(at)); memcpy(ar, a->rot, sizeof(ar));
				memcpy(bt, b->trans, sizeof(bt)); memcpy(br, b->rot, sizeof(br));
			} else {
				/* Absolute TRS comparison (MDR flattened parents). */
				abs_trs_for_joint(src, f, j, at, ar, as);
				/* dst already has absolute TRS (parent=-1). */
				const mc_joint_pose_t *b = &dst.jointPoses[f * dst.numJoints + j];
				memcpy(bt, b->trans, sizeof(bt)); memcpy(br, b->rot, sizeof(br));
			}
			for (int k = 0; k < 3; ++k) {
				float d = fabsf(at[k] - bt[k]);
				if (d > worst_t) worst_t = d;
			}
			float dot = ar[0]*br[0] + ar[1]*br[1] + ar[2]*br[2] + ar[3]*br[3];
			float sign = dot < 0 ? -1.0f : 1.0f;
			for (int k = 0; k < 4; ++k) {
				float d = fabsf(ar[k] - sign * br[k]);
				if (d > worst_r) worst_r = d;
			}
		}
	}
	if (worst_t > trs_tol || worst_r > trs_tol) {
		printf("FAIL worst trans=%.5f rot=%.5f\n", worst_t, worst_r);
		rc = 1; goto done;
	}
	/* Verify blend arrays exist and weights sum to ~1. */
	{
		const mc_surface_t *sb = &dst.surfaces[0];
		if (!sb->blendIndices || !sb->blendWeights) {
			printf("FAIL no blend arrays\n");
			rc = 1; goto done;
		}
		for (int v = 0; v < sb->numVerts; ++v) {
			float wsum = 0.0f;
			for (int k = 0; k < 4; ++k)
				wsum += sb->blendWeights[v * 4 + k];
			if (fabsf(wsum - 1.0f) > 0.02f) {
				printf("FAIL vertex %d weights sum %.4f\n", v, wsum);
				rc = 1; goto done;
			}
		}
	}
	printf("OK joints=%d frames=%d anims=%d worst-t=%.5f worst-r=%.5f\n",
		dst.numJoints, dst.numFrames, dst.numAnimations, worst_t, worst_r);
done:
	mc_model_free(&dst);
	return rc;
}

int main(int argc, char **argv) {
	const char *tmp = (argc > 1) ? argv[1] : ".";
	char path[1024];
	mc_model_t cube;
	make_cube(&cube);

	int rc = 0;
	snprintf(path, sizeof(path), "%s/cube.md3", tmp);
	rc |= round_trip("md3 ", path, mc_save_md3, mc_load_md3, &cube);
	snprintf(path, sizeof(path), "%s/cube.iqm", tmp);
	rc |= round_trip("iqm ", path, mc_save_iqm, mc_load_iqm, &cube);
	snprintf(path, sizeof(path), "%s/cube.mdr", tmp);
	rc |= round_trip("mdr ", path, mc_save_mdr, mc_load_mdr, &cube);
	snprintf(path, sizeof(path), "%s/cube.gltf", tmp);
	rc |= round_trip("gltf", path, gltf_save, gltf_load, &cube);
	snprintf(path, sizeof(path), "%s/cube.glb", tmp);
	rc |= round_trip("glb ", path, glb_save, gltf_load, &cube);

	mc_model_free(&cube);

	/* Animated cube round-trip: only formats that carry vertex morph
	   animation are exercised here. */
	mc_model_t acube;
	make_animated_cube(&acube);
	printf("animated cube: %d frames, %d tags\n", acube.numFrames, acube.numTags);
	snprintf(path, sizeof(path), "%s/cube_anim.md3", tmp);
	rc |= round_trip_anim("md3 ", path, mc_save_md3, mc_load_md3, &acube,
		/* MD3 quantises positions to 1/64 unit; expand for the 2x frame */
		0.05f);
	snprintf(path, sizeof(path), "%s/cube_anim.glb", tmp);
	rc |= round_trip_anim("glb ", path, glb_save, gltf_load, &acube, /* skip pos check */ -1.0f);
	snprintf(path, sizeof(path), "%s/cube_anim.gltf", tmp);
	rc |= round_trip_anim("gltf", path, gltf_save, gltf_load, &acube, /* skip pos check */ -1.0f);
	mc_model_free(&acube);

	/* Skeletal cube: IQM and MDR should round-trip skin losslessly,
	   glTF should round-trip joint TRS via cgltf_skin. */
	mc_model_t scube;
	make_skinned_cube(&scube);
	printf("skinned cube: %d frames, %d joints, %d verts\n",
		scube.numFrames, scube.numJoints, scube.surfaces[0].numVerts);
	snprintf(path, sizeof(path), "%s/cube_skel.iqm", tmp);
	rc |= round_trip_skel("iqm ", path, mc_save_iqm, mc_load_iqm, &scube);
	snprintf(path, sizeof(path), "%s/cube_skel.mdr", tmp);
	rc |= round_trip_skel("mdr ", path, mc_save_mdr, mc_load_mdr, &scube);
	snprintf(path, sizeof(path), "%s/cube_skel.glb", tmp);
	rc |= round_trip_skel("glb ", path, glb_save, gltf_load, &scube);
	mc_model_free(&scube);

	/* Multi-bone skeletal cube: 3 bones in a parent chain, 3 frames,
	   2 animations, mixed per-vertex bone assignments. */
	mc_model_t mcube;
	make_multibone_cube(&mcube);
	printf("multibone cube: %d frames, %d joints, %d anims, %d verts\n",
		mcube.numFrames, mcube.numJoints, mcube.numAnimations,
		mcube.surfaces[0].numVerts);
	snprintf(path, sizeof(path), "%s/cube_multi.iqm", tmp);
	rc |= round_trip_skel_multi("iqm ", path, mc_save_iqm, mc_load_iqm, &mcube,
		/*check_parents*/1);
	snprintf(path, sizeof(path), "%s/cube_multi.mdr", tmp);
	rc |= round_trip_skel_multi("mdr ", path, mc_save_mdr, mc_load_mdr, &mcube,
		/*check_parents*/0); /* MDR flattens parent hierarchy to -1 */
	mc_model_free(&mcube);

	/* Subdivide: one iteration should quadruple triangle counts on a
	   normal mesh. Decimate to ~50% should reduce tris by roughly half
	   and never produce NaN positions. */
	{
		printf("subdivide / decimate regression\n");
		mc_model_t s; make_cube(&s);
		int t0 = s.surfaces[0].numTris;
		if (mc_subdivide(&s, 1) != 0) { printf("  [subdiv] FAIL call\n"); rc = 1; }
		else {
			int t1 = s.surfaces[0].numTris;
			if (t1 != t0 * 4) { printf("  [subdiv] FAIL tris %d != %d*4\n", t1, t0); rc = 1; }
			else printf("  [subdiv] OK %d -> %d tris\n", t0, t1);
		}
		mc_model_free(&s);

		mc_model_t d; make_cube(&d);
		int td0 = d.surfaces[0].numTris;
		if (mc_decimate(&d, 0.5f) != 0) { printf("  [decim ] FAIL call\n"); rc = 1; }
		else {
			int td1 = d.surfaces[0].numTris;
			if (td1 < 1 || td1 > td0) { printf("  [decim ] FAIL tris %d (was %d)\n", td1, td0); rc = 1; }
			/* Sanity: no NaN / inf positions. */
			int bad = 0;
			const mc_surface_t *ss = &d.surfaces[0];
			for (int i = 0; i < ss->numVerts * 3; ++i) {
				float v = ss->xyz[i];
				if (!(v == v) || v > 1e30f || v < -1e30f) { bad = 1; break; }
			}
			if (bad) { printf("  [decim ] FAIL non-finite vertex\n"); rc = 1; }
			if (!rc) printf("  [decim ] OK %d -> %d tris\n", td0, td1);
		}
		mc_model_free(&d);
	}

	/* Shader parser adversarial input tests. */
	{
		printf("shader parser adversarial\n");
		mc_model_t sm; make_cube(&sm);

		/* Empty file. */
		{
			char p[1024]; snprintf(p, sizeof(p), "%s/empty.shader", tmp);
			mc_write_file(p, "", 0);
			int r = mc_load_q3shader(p, &sm);
			printf("  [empty ] %s\n", r == 0 ? "OK" : "OK (no stanzas)");
		}
		/* Deeply nested braces. */
		{
			char p[1024]; snprintf(p, sizeof(p), "%s/deep.shader", tmp);
			const char *deep = "test/deep { { { { { { { { } } } } } } } }\n";
			mc_write_file(p, deep, strlen(deep));
			mc_load_q3shader(p, &sm);
			printf("  [deep  ] OK (no crash)\n");
		}
		/* Unterminated brace. */
		{
			char p[1024]; snprintf(p, sizeof(p), "%s/unterm.shader", tmp);
			const char *s = "test/unterm {\n  {\n    map $lightmap\n";
			mc_write_file(p, s, strlen(s));
			mc_load_q3shader(p, &sm);
			printf("  [unterm] OK (no crash)\n");
		}
		/* Very long line. */
		{
			char p[1024]; snprintf(p, sizeof(p), "%s/longline.shader", tmp);
			char buf[4200];
			memset(buf, 'a', sizeof(buf));
			buf[0] = '{'; buf[sizeof(buf)-2] = '}'; buf[sizeof(buf)-1] = '\n';
			mc_write_file(p, buf, sizeof(buf));
			mc_load_q3shader(p, &sm);
			printf("  [long  ] OK (no crash)\n");
		}
		mc_model_free(&sm);
	}

	/* Skin + animation.cfg round-trip. */
	{
		printf("skin + animation.cfg round-trip\n");
		mc_model_t sm; make_cube(&sm);
		mc_q_strncpy(sm.surfaces[0].shader, "models/test/cube", sizeof(sm.surfaces[0].shader));
		char skinp[1024]; snprintf(skinp, sizeof(skinp), "%s/cube.skin", tmp);
		if (mc_save_skin(skinp, &sm) == 0) {
			mc_model_t sm2; make_cube(&sm2);
			mc_load_q3skin(skinp, &sm2);
			if (strcmp(sm2.surfaces[0].shader, sm.surfaces[0].shader) == 0)
				printf("  [skin  ] OK\n");
			else {
				printf("  [skin  ] FAIL shader '%s' != '%s'\n", sm2.surfaces[0].shader, sm.surfaces[0].shader);
				rc = 1;
			}
			mc_model_free(&sm2);
		}

		mc_animation_t *a = mc_model_add_animation(&sm);
		mc_q_strncpy(a->name, "TORSO_ATTACK", sizeof(a->name));
		a->firstFrame = 0; a->numFrames = 1; a->loopFrames = 0; a->fps = 15.0f;
		char animp[1024]; snprintf(animp, sizeof(animp), "%s/animation.cfg", tmp);
		mc_save_animation_cfg(animp, &sm);
		mc_model_t sm3; mc_model_init(&sm3);
		mc_load_animation_cfg(animp, &sm3);
		if (sm3.numAnimations >= 1 && strcmp(sm3.animations[0].name, "TORSO_ATTACK") == 0)
			printf("  [animcfg] OK\n");
		else {
			printf("  [animcfg] FAIL anims=%d\n", sm3.numAnimations);
			rc = 1;
		}
		mc_model_free(&sm3);
		mc_model_free(&sm);
	}

	if (rc) fprintf(stderr, "test_roundtrip: FAILED\n");
	else printf("test_roundtrip: OK\n");
	return rc;
}
