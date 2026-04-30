/*
===========================================================================
modelconverter - mesh subdivision

Loop subdivision applied to every surface and every animation frame of
the in-memory mc_model_t.  Loop is the triangle-mesh counterpart of
Catmull-Clark and produces the same kind of smooth, area-preserving
refinement while keeping the output a pure triangle mesh - which is the
only thing the MD3 / IQM / MDR writers can emit.

Per iteration each triangle is replaced by four sub-triangles using new
edge-midpoint vertices, and original vertices are moved towards the
average of their neighbours according to Loop's smoothing mask.  The
same topological refinement (vertex weights and triangle list) is built
once from frame 0 and then re-applied to every frame's POSITION buffer
so morph-target playback stays coherent.  UVs are linearly interpolated
on edges and left untouched on original verts; per-vertex normals are
recomputed per-frame from the new face normals.

Boundary edges (edges referenced by exactly one triangle) and boundary
vertices use the standard cubic-B-spline stencil so open edges (cape
fringes, hair strips, ...) stay attached and don't shrink.
===========================================================================
*/

#include "mc_common.h"

#include <math.h>
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Half-edge bookkeeping                                              */
/* ------------------------------------------------------------------ */

typedef struct {
	int v0, v1;	   /* endpoints, v0 < v1 */
	int opp0, opp1; /* opposite vertex of each adjacent triangle (-1 if none) */
	int newIndex;  /* index of the inserted edge-midpoint in the new vertex list */
} sd_edge_t;

typedef struct {
	int a, b, c;	  /* original vertex indices */
	int eAB, eBC, eCA; /* edge indices into edges[] */
} sd_tri_t;

/* ------------------------------------------------------------------ */
/* Open-addressing hash map: (uint64) edge key -> edge index         */
/* ------------------------------------------------------------------ */

typedef struct {
	uint64_t key;
	int value;
} sd_bucket_t;

typedef struct {
	sd_bucket_t *buckets;
	int cap;	/* power of 2 */
	int count;
} sd_hash_t;

static void sd_hash_init(sd_hash_t *h, int hint) {
	int cap = 16;
	while (cap < hint * 2)
		cap *= 2;
	h->cap = cap;
	h->count = 0;
	h->buckets = (sd_bucket_t *)mc_calloc((size_t)cap, sizeof(sd_bucket_t));
	for (int i = 0; i < cap; ++i)
		h->buckets[i].key = (uint64_t)-1;
}

static void sd_hash_free(sd_hash_t *h) {
	free(h->buckets);
	h->buckets = NULL;
}

static uint64_t sd_mix(uint64_t x) {
	x ^= x >> 33;
	x *= 0xff51afd7ed558ccdULL;
	x ^= x >> 33;
	x *= 0xc4ceb9fe1a85ec53ULL;
	x ^= x >> 33;
	return x;
}

/* Returns existing edge index for `key` or -1 if not present. */
static int sd_hash_get(const sd_hash_t *h, uint64_t key) {
	uint64_t mask = (uint64_t)(h->cap - 1);
	uint64_t idx = sd_mix(key) & mask;
	for (;;) {
		if (h->buckets[idx].key == (uint64_t)-1)
			return -1;
		if (h->buckets[idx].key == key)
			return h->buckets[idx].value;
		idx = (idx + 1) & mask;
	}
}

static void sd_hash_put(sd_hash_t *h, uint64_t key, int value) {
	uint64_t mask = (uint64_t)(h->cap - 1);
	uint64_t idx = sd_mix(key) & mask;
	for (;;) {
		if (h->buckets[idx].key == (uint64_t)-1) {
			h->buckets[idx].key = key;
			h->buckets[idx].value = value;
			h->count++;
			return;
		}
		if (h->buckets[idx].key == key) {
			h->buckets[idx].value = value;
			return;
		}
		idx = (idx + 1) & mask;
	}
}

static uint64_t sd_edge_key(int a, int b) {
	if (a > b) { int t = a; a = b; b = t; }
	return ((uint64_t)(uint32_t)a << 32) | (uint32_t)b;
}

/* ------------------------------------------------------------------ */
/* Cross-surface seam context.                                         */
/* Each MD3 surface is subdivided independently, but seams between     */
/* surfaces must produce identical positions in every surface that     */
/* touches them, otherwise cracks appear.  We index by quantised       */
/* world-space position (frame 0) so coincident verts/edges across     */
/* surfaces map to the same entry.                                     */
/* ------------------------------------------------------------------ */

typedef struct {
	uint64_t nbr_keys[4];
	float    nbr_pos[4][3];
	int      nbr_count; /* -1 means corner / non-manifold (>4 unique) */
} sd_seam_vert_t;

typedef struct {
	sd_hash_t       vmap;	  /* pos_key -> index into vverts */
	sd_seam_vert_t *vverts;
	int             vcount, vcap;
	sd_hash_t       eset;	  /* sorted (pos_key0,pos_key1) -> 1 */
} sd_seam_ctx_t;

static uint64_t sd_pos_key(const float p[3]) {
	int32_t x = (int32_t)lrintf(p[0] * 1024.0f);
	int32_t y = (int32_t)lrintf(p[1] * 1024.0f);
	int32_t z = (int32_t)lrintf(p[2] * 1024.0f);
	uint64_t k = (uint64_t)(uint32_t)x;
	k = sd_mix(k ^ (uint64_t)(uint32_t)y);
	k = sd_mix(k ^ (uint64_t)(uint32_t)z);
	if (k == 0 || k == (uint64_t)-1) k = 1;
	return k;
}

static uint64_t sd_pos_pair_key(uint64_t a, uint64_t b) {
	if (a > b) { uint64_t t = a; a = b; b = t; }
	return sd_mix(a) ^ b;
}

static void sd_seam_init(sd_seam_ctx_t *ctx, int hint) {
	int h = hint > 16 ? hint : 16;
	sd_hash_init(&ctx->vmap, h);
	sd_hash_init(&ctx->eset, h);
	ctx->vverts = NULL; ctx->vcount = 0; ctx->vcap = 0;
}

static void sd_seam_free(sd_seam_ctx_t *ctx) {
	sd_hash_free(&ctx->vmap);
	sd_hash_free(&ctx->eset);
	free(ctx->vverts);
	ctx->vverts = NULL;
}

static int sd_seam_vget(sd_seam_ctx_t *ctx, uint64_t key) {
	int idx = sd_hash_get(&ctx->vmap, key);
	if (idx >= 0) return idx;
	if (ctx->vcount >= ctx->vcap) {
		ctx->vcap = ctx->vcap ? ctx->vcap * 2 : 64;
		ctx->vverts = (sd_seam_vert_t *)mc_realloc(ctx->vverts,
			sizeof(sd_seam_vert_t) * (size_t)ctx->vcap);
	}
	idx = ctx->vcount++;
	ctx->vverts[idx].nbr_count = 0;
	sd_hash_put(&ctx->vmap, key, idx);
	return idx;
}

static void sd_seam_add_nbr(sd_seam_ctx_t *ctx, const float self[3], const float nbr[3]) {
	uint64_t sk = sd_pos_key(self);
	uint64_t nk = sd_pos_key(nbr);
	int idx = sd_seam_vget(ctx, sk);
	sd_seam_vert_t *e = &ctx->vverts[idx];
	if (e->nbr_count < 0) return;
	for (int i = 0; i < e->nbr_count; ++i)
		if (e->nbr_keys[i] == nk) return;
	if (e->nbr_count >= 4) { e->nbr_count = -1; return; }
	e->nbr_keys[e->nbr_count] = nk;
	e->nbr_pos[e->nbr_count][0] = nbr[0];
	e->nbr_pos[e->nbr_count][1] = nbr[1];
	e->nbr_pos[e->nbr_count][2] = nbr[2];
	e->nbr_count++;
}

static void sd_seam_add_edge(sd_seam_ctx_t *ctx, const float p0[3], const float p1[3]) {
	uint64_t k = sd_pos_pair_key(sd_pos_key(p0), sd_pos_key(p1));
	sd_hash_put(&ctx->eset, k, 1);
}

static int sd_seam_has_edge(const sd_seam_ctx_t *ctx, const float p0[3], const float p1[3]) {
	uint64_t k = sd_pos_pair_key(sd_pos_key(p0), sd_pos_key(p1));
	return sd_hash_get(&ctx->eset, k) >= 0;
}

/* Build the model-wide seam context from frame-0 positions of every
   surface's boundary edges (an edge belongs to exactly one triangle
   inside its surface).  Each boundary edge contributes the edge to
   the seam edge set and both endpoints/neighbours to the seam vert
   map. */
static void sd_seam_build(sd_seam_ctx_t *ctx, mc_model_t *m) {
	int hint = 0;
	for (int si = 0; si < m->numSurfaces; ++si)
		hint += m->surfaces[si].numTris * 3;
	sd_seam_init(ctx, hint);

	for (int si = 0; si < m->numSurfaces; ++si) {
		mc_surface_t *s = &m->surfaces[si];
		int nv = s->numVerts, nt = s->numTris;
		if (nv <= 0 || nt <= 0) continue;

		sd_hash_t emap;
		sd_hash_init(&emap, nt * 3);
		typedef struct { int v0, v1, count; } sd_eb_t;
		sd_eb_t *eb = (sd_eb_t *)mc_malloc(sizeof(sd_eb_t) * (size_t)nt * 3);
		int ne = 0;
		for (int t = 0; t < nt; ++t) {
			int idx[3] = { s->indices[t * 3 + 0],
						   s->indices[t * 3 + 1],
						   s->indices[t * 3 + 2] };
			for (int k = 0; k < 3; ++k) {
				int a = idx[k], b = idx[(k + 1) % 3];
				uint64_t key = sd_edge_key(a, b);
				int ei = sd_hash_get(&emap, key);
				if (ei < 0) {
					ei = ne++;
					eb[ei].v0 = a < b ? a : b;
					eb[ei].v1 = a < b ? b : a;
					eb[ei].count = 1;
					sd_hash_put(&emap, key, ei);
				} else {
					eb[ei].count++;
				}
			}
		}
		const float *xyz = s->xyz; /* frame 0 */
		for (int e = 0; e < ne; ++e) {
			if (eb[e].count != 1) continue;
			const float *p0 = &xyz[eb[e].v0 * 3];
			const float *p1 = &xyz[eb[e].v1 * 3];
			sd_seam_add_nbr(ctx, p0, p1);
			sd_seam_add_nbr(ctx, p1, p0);
			sd_seam_add_edge(ctx, p0, p1);
		}
		free(eb);
		sd_hash_free(&emap);
	}
}

/* Look up a seam vertex by position; returns NULL if not on a seam. */
static const sd_seam_vert_t *sd_seam_lookup(const sd_seam_ctx_t *ctx, const float p[3]) {
	int idx = sd_hash_get(&ctx->vmap, sd_pos_key(p));
	if (idx < 0) return NULL;
	return &ctx->vverts[idx];
}

/* ------------------------------------------------------------------ */
/* Subdivide one surface in-place.                                    */
/* ------------------------------------------------------------------ */

static void sd_face_normal(const float *p0, const float *p1, const float *p2, float n[3]) {
	float ax = p1[0] - p0[0], ay = p1[1] - p0[1], az = p1[2] - p0[2];
	float bx = p2[0] - p0[0], by = p2[1] - p0[1], bz = p2[2] - p0[2];
	n[0] = ay * bz - az * by;
	n[1] = az * bx - ax * bz;
	n[2] = ax * by - ay * bx;
}

static void sd_normalize(float v[3]) {
	float l = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	if (l > 1e-20f) { v[0] /= l; v[1] /= l; v[2] /= l; }
}

static int subdivide_surface_once(mc_surface_t *s, int numFrames,
								  const sd_seam_ctx_t *seam) {
	const int oldVerts = s->numVerts;
	const int oldTris = s->numTris;
	if (oldVerts <= 0 || oldTris <= 0)
		return 0;

	/* ---- 1. Collect unique edges & per-tri edge indices. ---- */
	sd_tri_t *tris = (sd_tri_t *)mc_malloc(sizeof(sd_tri_t) * (size_t)oldTris);
	sd_edge_t *edges = (sd_edge_t *)mc_malloc(sizeof(sd_edge_t) * (size_t)oldTris * 3);
	int numEdges = 0;
	sd_hash_t map;
	sd_hash_init(&map, oldTris * 3);

	for (int t = 0; t < oldTris; ++t) {
		int a = s->indices[t * 3 + 0];
		int b = s->indices[t * 3 + 1];
		int c = s->indices[t * 3 + 2];
		tris[t].a = a; tris[t].b = b; tris[t].c = c;
		int *outE[3] = {&tris[t].eAB, &tris[t].eBC, &tris[t].eCA};
		int pairs[3][3] = {{a, b, c}, {b, c, a}, {c, a, b}};
		for (int k = 0; k < 3; ++k) {
			int v0 = pairs[k][0], v1 = pairs[k][1], opp = pairs[k][2];
			uint64_t key = sd_edge_key(v0, v1);
			int ei = sd_hash_get(&map, key);
			if (ei < 0) {
				ei = numEdges++;
				edges[ei].v0 = v0 < v1 ? v0 : v1;
				edges[ei].v1 = v0 < v1 ? v1 : v0;
				edges[ei].opp0 = opp;
				edges[ei].opp1 = -1;
				edges[ei].newIndex = -1;
				sd_hash_put(&map, key, ei);
			} else {
				if (edges[ei].opp1 == -1) {
					edges[ei].opp1 = opp;
				} else {
					/* >2 triangles per edge: non-manifold; ignore extra. */
					MC_LOG("subdiv: surface '%s' has non-manifold edge %d-%d "
						"(>2 incident triangles, dropping extra at tri %d) - "
						"hint: weld duplicate verts or split overlapping shells in your DCC tool\n",
						s->name, edges[ei].v0, edges[ei].v1, t);
				}
			}
			*outE[k] = ei;
		}
	}
	sd_hash_free(&map);

	/* ---- 2. Per-vertex neighbour lists & boundary classification.
	   Neighbours are vertices reached by an outgoing edge (boundary or
	   interior).  Boundary vertices are those touched by any boundary
	   edge.  We accumulate sums of neighbour positions in a separate
	   pass per frame; valence stays the same across frames. */
	int *valence = (int *)mc_calloc((size_t)oldVerts, sizeof(int));
	int *boundary = (int *)mc_calloc((size_t)oldVerts, sizeof(int));
	/* For boundary vertices we need just the two boundary neighbours.
	   Store them in bndN[v*2 + 0/1]; -1 marks unused. */
	int *bndN = (int *)mc_malloc(sizeof(int) * (size_t)oldVerts * 2);
	for (int i = 0; i < oldVerts * 2; ++i) bndN[i] = -1;

	/* Build adjacency: per-vertex neighbour list (CSR). */
	int *adjCnt = (int *)mc_calloc((size_t)oldVerts, sizeof(int));
	for (int e = 0; e < numEdges; ++e) {
		adjCnt[edges[e].v0]++;
		adjCnt[edges[e].v1]++;
	}
	int *adjOfs = (int *)mc_malloc(sizeof(int) * (size_t)(oldVerts + 1));
	adjOfs[0] = 0;
	for (int i = 0; i < oldVerts; ++i) adjOfs[i + 1] = adjOfs[i] + adjCnt[i];
	int *adj = (int *)mc_malloc(sizeof(int) * (size_t)adjOfs[oldVerts]);
	int *fillCnt = (int *)mc_calloc((size_t)oldVerts, sizeof(int));
	for (int e = 0; e < numEdges; ++e) {
		int v0 = edges[e].v0, v1 = edges[e].v1;
		adj[adjOfs[v0] + fillCnt[v0]++] = v1;
		adj[adjOfs[v1] + fillCnt[v1]++] = v0;
	}
	free(fillCnt);
	for (int i = 0; i < oldVerts; ++i) valence[i] = adjCnt[i];
	free(adjCnt);

	for (int e = 0; e < numEdges; ++e) {
		int isBnd = (edges[e].opp1 == -1);
		if (isBnd) {
			int v0 = edges[e].v0, v1 = edges[e].v1;
			boundary[v0] = 1;
			boundary[v1] = 1;
			if (bndN[v0 * 2] == -1) bndN[v0 * 2] = v1; else bndN[v0 * 2 + 1] = v1;
			if (bndN[v1 * 2] == -1) bndN[v1 * 2] = v0; else bndN[v1 * 2 + 1] = v0;
		}
	}

	/* ---- 3. Allocate the new mesh. ---- */
	int newVerts = oldVerts + numEdges;
	int newTris = oldTris * 4;

	/* Assign newIndex to each edge midpoint. */
	for (int e = 0; e < numEdges; ++e)
		edges[e].newIndex = oldVerts + e;

	float *newXYZ = (float *)mc_malloc(sizeof(float) * (size_t)numFrames * (size_t)newVerts * 3);
	float *newST = (float *)mc_malloc(sizeof(float) * (size_t)newVerts * 2);
	int *newIdx = (int *)mc_malloc(sizeof(int) * (size_t)newTris * 3);

	/* ---- 4. UVs (frame-independent). ---- */
	if (s->st) {
		memcpy(newST, s->st, sizeof(float) * 2 * (size_t)oldVerts);
		for (int e = 0; e < numEdges; ++e) {
			const float *u0 = &s->st[edges[e].v0 * 2];
			const float *u1 = &s->st[edges[e].v1 * 2];
			float *du = &newST[edges[e].newIndex * 2];
			du[0] = 0.5f * (u0[0] + u1[0]);
			du[1] = 0.5f * (u0[1] + u1[1]);
		}
	} else {
		memset(newST, 0, sizeof(float) * 2 * (size_t)newVerts);
	}

	/* ---- 5. Per-frame XYZ subdivision (Loop masks). ---- */
	for (int f = 0; f < numFrames; ++f) {
		const float *src = &s->xyz[(size_t)f * oldVerts * 3];
		float *dst = &newXYZ[(size_t)f * newVerts * 3];

		/* (a) New edge midpoints. */
		for (int e = 0; e < numEdges; ++e) {
			const float *p0 = &src[edges[e].v0 * 3];
			const float *p1 = &src[edges[e].v1 * 3];
			float *o = &dst[edges[e].newIndex * 3];
			int isBnd = (edges[e].opp1 == -1) ||
						(seam && sd_seam_has_edge(seam, p0, p1));
			if (isBnd) {
				/* Boundary edge (here or in another surface): use the
				   simple midpoint so the new vertex matches across
				   every surface that subdivides this edge. */
				o[0] = 0.5f * (p0[0] + p1[0]);
				o[1] = 0.5f * (p0[1] + p1[1]);
				o[2] = 0.5f * (p0[2] + p1[2]);
			} else {
				const float *q0 = &src[edges[e].opp0 * 3];
				const float *q1 = &src[edges[e].opp1 * 3];
				/* Loop edge mask: 3/8 (p0+p1) + 1/8 (q0+q1). */
				o[0] = 0.375f * (p0[0] + p1[0]) + 0.125f * (q0[0] + q1[0]);
				o[1] = 0.375f * (p0[1] + p1[1]) + 0.125f * (q0[1] + q1[1]);
				o[2] = 0.375f * (p0[2] + p1[2]) + 0.125f * (q0[2] + q1[2]);
			}
		}

		/* (b) Smooth original vertices.

		   Verts whose position appears in the cross-surface seam map
		   are smoothed using that global view (all surfaces touching
		   them produce the same result, so seams stay watertight).
		   Verts not on any seam use the standard interior Loop mask. */
		for (int v = 0; v < oldVerts; ++v) {
			const float *p = &src[v * 3];
			float *o = &dst[v * 3];
			const sd_seam_vert_t *sv = seam ? sd_seam_lookup(seam, p) : NULL;
			if (sv) {
				if (sv->nbr_count == 2) {
					/* Loop boundary mask: 3/4 self + 1/8 each. */
					o[0] = 0.75f * p[0] + 0.125f * (sv->nbr_pos[0][0] + sv->nbr_pos[1][0]);
					o[1] = 0.75f * p[1] + 0.125f * (sv->nbr_pos[0][1] + sv->nbr_pos[1][1]);
					o[2] = 0.75f * p[2] + 0.125f * (sv->nbr_pos[0][2] + sv->nbr_pos[1][2]);
				} else {
					/* Corner / non-manifold: pin in place. */
					o[0] = p[0]; o[1] = p[1]; o[2] = p[2];
				}
			} else if (boundary[v]) {
				/* Open boundary not shared with another surface: still
				   apply Loop boundary smoothing using local nbrs. */
				int n0 = bndN[v * 2], n1 = bndN[v * 2 + 1];
				if (n0 < 0 || n1 < 0) {
					o[0] = p[0]; o[1] = p[1]; o[2] = p[2];
				} else {
					const float *a = &src[n0 * 3];
					const float *b = &src[n1 * 3];
					o[0] = 0.75f * p[0] + 0.125f * (a[0] + b[0]);
					o[1] = 0.75f * p[1] + 0.125f * (a[1] + b[1]);
					o[2] = 0.75f * p[2] + 0.125f * (a[2] + b[2]);
				}
			} else {
				int n = valence[v];
				/* Loop's β: (1/n) * (5/8 - (3/8 + 1/4 cos(2π/n))²). */
				float c = 0.375f + 0.25f * cosf(6.28318530718f / (float)n);
				float beta = (0.625f - c * c) / (float)n;
				float sx = 0, sy = 0, sz = 0;
				for (int k = adjOfs[v]; k < adjOfs[v + 1]; ++k) {
					const float *q = &src[adj[k] * 3];
					sx += q[0]; sy += q[1]; sz += q[2];
				}
				float self = 1.0f - (float)n * beta;
				o[0] = self * p[0] + beta * sx;
				o[1] = self * p[1] + beta * sy;
				o[2] = self * p[2] + beta * sz;
			}
		}
	}

	/* ---- 6. New triangle list (4 sub-tris per old). ---- */
	for (int t = 0; t < oldTris; ++t) {
		int a = tris[t].a, b = tris[t].b, c = tris[t].c;
		int mAB = edges[tris[t].eAB].newIndex;
		int mBC = edges[tris[t].eBC].newIndex;
		int mCA = edges[tris[t].eCA].newIndex;
		int *o = &newIdx[t * 12];
		o[0] = a;	o[1] = mAB; o[2] = mCA;
		o[3] = b;	o[4] = mBC; o[5] = mAB;
		o[6] = c;	o[7] = mCA; o[8] = mBC;
		o[9] = mAB; o[10] = mBC; o[11] = mCA;
	}

	/* ---- 7. Per-frame normals.  We propagate the original
	   author-controlled per-vertex normals (which may not match the
	   geometric face cross product - MD3 winding can be CW while
	   normals point outward) to surviving vertices and average the
	   two endpoint normals for each new edge midpoint, renormalising
	   afterwards.  This preserves smooth shading and correct
	   front-facing orientation across all formats. */
	float *newN = (float *)mc_calloc((size_t)numFrames * (size_t)newVerts * 3, sizeof(float));
	if (s->normal) {
		for (int f = 0; f < numFrames; ++f) {
			const float *srcN = &s->normal[(size_t)f * oldVerts * 3];
			float *dstN = &newN[(size_t)f * newVerts * 3];
			memcpy(dstN, srcN, sizeof(float) * 3 * (size_t)oldVerts);
			for (int e = 0; e < numEdges; ++e) {
				const float *n0 = &srcN[edges[e].v0 * 3];
				const float *n1 = &srcN[edges[e].v1 * 3];
				float *o = &dstN[edges[e].newIndex * 3];
				o[0] = n0[0] + n1[0];
				o[1] = n0[1] + n1[1];
				o[2] = n0[2] + n1[2];
				sd_normalize(o);
			}
		}
	} else {
		/* No source normals: fall back to geometric face normals. */
		for (int f = 0; f < numFrames; ++f) {
			float *frameXYZ = &newXYZ[(size_t)f * newVerts * 3];
			float *frameN = &newN[(size_t)f * newVerts * 3];
			for (int t = 0; t < newTris; ++t) {
				int ia = newIdx[t * 3 + 0];
				int ib = newIdx[t * 3 + 1];
				int ic = newIdx[t * 3 + 2];
				float fn[3];
				sd_face_normal(&frameXYZ[ia * 3], &frameXYZ[ib * 3], &frameXYZ[ic * 3], fn);
				for (int k = 0; k < 3; ++k) {
					frameN[ia * 3 + k] += fn[k];
					frameN[ib * 3 + k] += fn[k];
					frameN[ic * 3 + k] += fn[k];
				}
			}
			for (int v = 0; v < newVerts; ++v)
				sd_normalize(&frameN[v * 3]);
		}
	}

	/* ---- 8. Swap buffers. ---- */
	free(s->xyz); free(s->normal); free(s->st); free(s->indices);
	s->xyz = newXYZ;
	s->normal = newN;
	s->st = newST;
	s->indices = newIdx;
	s->numVerts = newVerts;
	s->numTris = newTris;

	free(tris);
	free(edges);
	free(valence);
	free(boundary);
	free(bndN);
	free(adjOfs);
	free(adj);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Public entry point                                                 */
/* ------------------------------------------------------------------ */

int mc_subdivide(mc_model_t *m, int iterations) {
	if (!m || iterations <= 0)
		return 0;
	/* Track which surfaces have been frozen because subdividing them
	   further would exceed the engine's per-surface vertex cap
	   (SHADER_MAX_VERTEXES = 1000). The engine
	   warns when a surface has >= 1000 verts; cap at 999 to stay
	   under that. */
	enum { MC_SUBDIV_MAX_VERTS = 999 };
	char *frozen = (char *)mc_calloc((size_t)m->numSurfaces, 1);
	if (!frozen)
		return -1;
	for (int it = 0; it < iterations; ++it) {
		/* Build a fresh seam context for this iteration: positions
		   change after each subdivision pass. */
		sd_seam_ctx_t seam;
		sd_seam_build(&seam, m);
		for (int i = 0; i < m->numSurfaces; ++i) {
			mc_surface_t *s = &m->surfaces[i];
			if (s->deform[0]) {
				/* autosprite / autosprite2 require the original quad layout to
				   render correctly; subdividing them would break the camera-
				   facing math. Pass through unchanged - this is the correct
				   action, not an error. */
				MC_DEBUG("subdiv: surface %d '%s' shader='%s' kept as-is (deformVertexes %s requires quad layout)\n",
						 i, s->name, s->shader, s->deform);
				continue;
			}
			if (frozen[i])
				continue;
			/* Loop subdivision adds exactly one new vertex per unique
			   edge, so the post-iteration vertex count is
			   oldV + numUniqueEdges. Count edges via a tiny open hash
			   keyed on the canonicalised endpoint pair. */
			int oldV = s->numVerts, oldT = s->numTris;
			int est;
			{
				size_t cap = 1;
				while (cap < (size_t)(oldT * 4) + 16)
					cap <<= 1;
				uint64_t *table = (uint64_t *)mc_calloc(cap, sizeof(uint64_t));
				if (!table) {
					sd_seam_free(&seam);
					free(frozen);
					return -1;
				}
				int unique = 0;
				for (int t = 0; t < oldT; ++t) {
					int idx[3] = { s->indices[t * 3], s->indices[t * 3 + 1], s->indices[t * 3 + 2] };
					for (int e = 0; e < 3; ++e) {
						int a = idx[e], b = idx[(e + 1) % 3];
						if (a > b) { int tmp = a; a = b; b = tmp; }
						uint64_t key = ((uint64_t)(uint32_t)a << 32) | (uint64_t)(uint32_t)b;
						/* key=0 (a=0,b=0) is impossible for a real edge, so
						   we use 0 as the empty sentinel. Bias by +1 to
						   guarantee non-zero. */
						uint64_t store = key + 1ULL;
						size_t h = (size_t)((key * 11400714819323198485ULL) >> 1) & (cap - 1);
						while (table[h] && table[h] != store)
							h = (h + 1) & (cap - 1);
						if (!table[h]) {
							table[h] = store;
							++unique;
						}
					}
				}
				free(table);
				est = oldV + unique;
			}
			if (est > MC_SUBDIV_MAX_VERTS) {
				MC_LOG("subdiv: freezing surface %d '%s' at iter %d (verts=%d, next=%d > %d cap)\n",
					   i, s->name, it + 1, oldV, est, MC_SUBDIV_MAX_VERTS);
				frozen[i] = 1;
				continue;
			}
			if (subdivide_surface_once(s, m->numFrames, &seam) != 0) {
				sd_seam_free(&seam);
				free(frozen);
				return -1;
			}
			MC_LOG("subdiv: surface %d '%s' iter %d/%d  verts %d->%d  tris %d->%d\n",
				   i, s->name, it + 1, iterations, oldV, s->numVerts, oldT, s->numTris);
		}
		sd_seam_free(&seam);
		/* Bounds may have shifted slightly; refresh per-frame bounds. */
		if (m->frames) {
			for (int f = 0; f < m->numFrames; ++f) {
				float mins[3] = { 1e30f, 1e30f, 1e30f };
				float maxs[3] = {-1e30f,-1e30f,-1e30f};
				for (int si = 0; si < m->numSurfaces; ++si) {
					const mc_surface_t *s = &m->surfaces[si];
					const float *xyz = &s->xyz[(size_t)f * s->numVerts * 3];
					for (int v = 0; v < s->numVerts; ++v) {
						for (int k = 0; k < 3; ++k) {
							if (xyz[v * 3 + k] < mins[k]) mins[k] = xyz[v * 3 + k];
							if (xyz[v * 3 + k] > maxs[k]) maxs[k] = xyz[v * 3 + k];
						}
					}
				}
				if (m->numSurfaces > 0) {
					memcpy(m->frames[f].bounds[0], mins, sizeof(mins));
					memcpy(m->frames[f].bounds[1], maxs, sizeof(maxs));
					float r = 0.0f;
					for (int k = 0; k < 3; ++k) {
						float c = 0.5f * (mins[k] + maxs[k]);
						float d = maxs[k] - c;
						r += d * d;
					}
					m->frames[f].radius = sqrtf(r);
				}
			}
		}
	}
	free(frozen);
	return 0;
}
