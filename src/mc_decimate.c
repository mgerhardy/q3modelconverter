/*
===========================================================================
modelconverter - mesh decimation (Garland-Heckbert quadric error metric)

Greedy edge-collapse decimation. For each surface we

  1. accumulate a 4x4 symmetric "quadric" matrix per vertex from the
     plane equations of all incident faces (weighted by face area);
  2. for every unique edge, find the parameter t in [0,1] along the
     edge that minimises the combined quadric error and queue
     (cost, v0, v1) in a min-heap;
  3. repeatedly pop the cheapest edge, collapse v1 into v0 by
     interpolating its position with parameter t (per-frame), merge
     quadrics, kill triangles that became degenerate, and re-queue the
     affected neighbour edges;
  4. compact the vertex / index arrays once the target triangle count
     has been reached.

For animated meshes the same collapse sequence and the same edge
parameter `t` are reused for every frame, so morph-target playback
stays coherent.  UVs are linearly interpolated, normals are recomputed
per frame from the new geometry.  Boundary edges (open edges referenced
by exactly one triangle) get a virtual perpendicular plane added to
both endpoint quadrics so the silhouette of capes / hair strips / open
shells is preserved.
===========================================================================
*/

#include "mc_common.h"

#include <math.h>
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Quadric (symmetric 4x4 stored as 10 doubles)                       */
/*    q = [ q0 q1 q2 q3                                               */
/*          q1 q4 q5 q6                                               */
/*          q2 q5 q7 q8                                               */
/*          q3 q6 q8 q9 ]                                             */
/* ------------------------------------------------------------------ */

typedef struct { double q[10]; } qem_t;

static void qem_zero(qem_t *q) {
	for (int i = 0; i < 10; ++i) q->q[i] = 0.0;
}

static void qem_add(qem_t *dst, const qem_t *src) {
	for (int i = 0; i < 10; ++i) dst->q[i] += src->q[i];
}

static void qem_add_plane(qem_t *q, double a, double b, double c, double d, double w) {
	q->q[0] += w * a * a;
	q->q[1] += w * a * b;
	q->q[2] += w * a * c;
	q->q[3] += w * a * d;
	q->q[4] += w * b * b;
	q->q[5] += w * b * c;
	q->q[6] += w * b * d;
	q->q[7] += w * c * c;
	q->q[8] += w * c * d;
	q->q[9] += w * d * d;
}

static double qem_eval(const qem_t *q, double x, double y, double z) {
	return q->q[0] * x * x + 2.0 * q->q[1] * x * y + 2.0 * q->q[2] * x * z + 2.0 * q->q[3] * x +
		   q->q[4] * y * y + 2.0 * q->q[5] * y * z + 2.0 * q->q[6] * y +
		   q->q[7] * z * z + 2.0 * q->q[8] * z + q->q[9];
}

/* ------------------------------------------------------------------ */
/* Min-heap of (cost, v0, v1)                                         */
/* ------------------------------------------------------------------ */

typedef struct { double cost; int v0, v1; } heap_entry_t;

typedef struct {
	heap_entry_t *items;
	int count, cap;
} heap_t;

static void heap_init(heap_t *h, int hint) {
	h->cap = hint > 16 ? hint : 16;
	h->count = 0;
	h->items = (heap_entry_t *)malloc(sizeof(heap_entry_t) * (size_t)h->cap);
}

static void heap_free(heap_t *h) {
	free(h->items);
	h->items = NULL;
}

static void heap_push(heap_t *h, double cost, int v0, int v1) {
	if (h->count == h->cap) {
		h->cap *= 2;
		h->items = (heap_entry_t *)realloc(h->items, sizeof(heap_entry_t) * (size_t)h->cap);
	}
	int i = h->count++;
	h->items[i].cost = cost;
	h->items[i].v0 = v0;
	h->items[i].v1 = v1;
	while (i > 0) {
		int p = (i - 1) >> 1;
		if (h->items[p].cost <= h->items[i].cost) break;
		heap_entry_t tmp = h->items[p];
		h->items[p] = h->items[i];
		h->items[i] = tmp;
		i = p;
	}
}

static int heap_pop(heap_t *h, heap_entry_t *out) {
	if (h->count == 0) return 0;
	*out = h->items[0];
	h->count--;
	if (h->count > 0) {
		h->items[0] = h->items[h->count];
		int i = 0;
		for (;;) {
			int l = i * 2 + 1, r = l + 1, m = i;
			if (l < h->count && h->items[l].cost < h->items[m].cost) m = l;
			if (r < h->count && h->items[r].cost < h->items[m].cost) m = r;
			if (m == i) break;
			heap_entry_t tmp = h->items[m];
			h->items[m] = h->items[i];
			h->items[i] = tmp;
			i = m;
		}
	}
	return 1;
}

/* ------------------------------------------------------------------ */
/* Per-vertex incident triangle list (small dynamic array)            */
/* ------------------------------------------------------------------ */

typedef struct {
	int *items;
	int count, cap;
} int_vec_t;

static void iv_push(int_vec_t *v, int x) {
	if (v->count == v->cap) {
		v->cap = v->cap ? v->cap * 2 : 4;
		v->items = (int *)realloc(v->items, sizeof(int) * (size_t)v->cap);
	}
	v->items[v->count++] = x;
}

static void iv_remove(int_vec_t *v, int x) {
	for (int i = 0; i < v->count; ++i) {
		if (v->items[i] == x) {
			v->items[i] = v->items[--v->count];
			return;
		}
	}
}

static int iv_contains(const int_vec_t *v, int x) {
	for (int i = 0; i < v->count; ++i) if (v->items[i] == x) return 1;
	return 0;
}

/* ------------------------------------------------------------------ */
/* Edge collapse cost: optimum t in [0,1] along the edge              */
/* ------------------------------------------------------------------ */

static double edge_optimum(const qem_t *qSum, const float *p0, const float *p1, double *outT) {
	double dx = (double)p1[0] - p0[0];
	double dy = (double)p1[1] - p0[1];
	double dz = (double)p1[2] - p0[2];
	/* Sample the quadric at t = 0, 0.5, 1 to recover the
	   coefficients of the resulting 1-D parabola in t. */
	double c0 = qem_eval(qSum, p0[0], p0[1], p0[2]);
	double c1 = qem_eval(qSum, p0[0] + 0.5 * dx, p0[1] + 0.5 * dy, p0[2] + 0.5 * dz);
	double c2 = qem_eval(qSum, p1[0], p1[1], p1[2]);
	double A = 2.0 * c0 + 2.0 * c2 - 4.0 * c1;
	double B = 4.0 * c1 - 3.0 * c0 - c2;
	double t = 0.5;
	if (A > 1e-20) {
		t = -B / (2.0 * A);
		if (t < 0.0) t = 0.0;
		else if (t > 1.0) t = 1.0;
	} else {
		/* Degenerate (linear / constant) - pick whichever endpoint
		   is cheaper. */
		t = (c0 <= c2) ? 0.0 : 1.0;
	}
	double cost = A * t * t + B * t + c0;
	if (cost < 0.0) cost = 0.0; /* numerical safety */
	*outT = t;
	return cost;
}

/* ------------------------------------------------------------------ */
/* Decimate a single surface in place                                 */
/* ------------------------------------------------------------------ */

static void normalize3(double v[3]) {
	double l = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	if (l > 1e-30) { v[0] /= l; v[1] /= l; v[2] /= l; }
}

static int decimate_surface(mc_surface_t *s, int numFrames, int targetTris) {
	const int oldVerts = s->numVerts;
	const int oldTris = s->numTris;
	if (targetTris >= oldTris || oldTris <= 1) return 0;
	if (targetTris < 1) targetTris = 1;

	/* --- Allocate working state ------------------------------------ */
	qem_t *Q = (qem_t *)calloc((size_t)oldVerts, sizeof(qem_t));
	int_vec_t *vertTris = (int_vec_t *)calloc((size_t)oldVerts, sizeof(int_vec_t));
	int *triAlive = (int *)malloc(sizeof(int) * (size_t)oldTris);
	int *triV = (int *)malloc(sizeof(int) * (size_t)oldTris * 3);
	int *vertAlive = (int *)malloc(sizeof(int) * (size_t)oldVerts);
	int *vertParent = (int *)malloc(sizeof(int) * (size_t)oldVerts);
	for (int i = 0; i < oldVerts; ++i) { vertAlive[i] = 1; vertParent[i] = -1; }
	for (int i = 0; i < oldTris; ++i) triAlive[i] = 1;
	memcpy(triV, s->indices, sizeof(int) * (size_t)oldTris * 3);

	/* Per-vertex (frame_count x 3) positions, accumulated independently
	   per frame - this is the buffer we update in place during
	   collapses and read back at the end. */
	float *xyz = (float *)malloc(sizeof(float) * (size_t)numFrames * (size_t)oldVerts * 3);
	memcpy(xyz, s->xyz, sizeof(float) * (size_t)numFrames * (size_t)oldVerts * 3);
	float *nrm = NULL;
	if (s->normal) {
		nrm = (float *)malloc(sizeof(float) * (size_t)numFrames * (size_t)oldVerts * 3);
		memcpy(nrm, s->normal, sizeof(float) * (size_t)numFrames * (size_t)oldVerts * 3);
	}
	float *st = NULL;
	if (s->st) {
		st = (float *)malloc(sizeof(float) * 2 * (size_t)oldVerts);
		memcpy(st, s->st, sizeof(float) * 2 * (size_t)oldVerts);
	}

	/* --- Build face quadrics (frame 0) and adjacency --------------- */
	const float *frame0 = s->xyz;
	for (int t = 0; t < oldTris; ++t) {
		int a = triV[t * 3 + 0], b = triV[t * 3 + 1], c = triV[t * 3 + 2];
		const float *pa = &frame0[a * 3], *pb = &frame0[b * 3], *pc = &frame0[c * 3];
		double ux = pb[0] - pa[0], uy = pb[1] - pa[1], uz = pb[2] - pa[2];
		double vx = pc[0] - pa[0], vy = pc[1] - pa[1], vz = pc[2] - pa[2];
		double nx = uy * vz - uz * vy;
		double ny = uz * vx - ux * vz;
		double nz = ux * vy - uy * vx;
		double area2 = sqrt(nx * nx + ny * ny + nz * nz);
		if (area2 < 1e-20) {
			triAlive[t] = 0; /* degenerate input */
			continue;
		}
		double inv = 1.0 / area2;
		nx *= inv; ny *= inv; nz *= inv;
		double d = -(nx * pa[0] + ny * pa[1] + nz * pa[2]);
		double w = 0.5 * area2; /* triangle area */
		qem_t plane;
		qem_zero(&plane);
		qem_add_plane(&plane, nx, ny, nz, d, w);
		qem_add(&Q[a], &plane);
		qem_add(&Q[b], &plane);
		qem_add(&Q[c], &plane);
		iv_push(&vertTris[a], t);
		iv_push(&vertTris[b], t);
		iv_push(&vertTris[c], t);
	}

	/* --- Boundary preservation: detect edges referenced by exactly
		one alive triangle and add a strong perpendicular plane to the
		quadric of both endpoints.  We iterate edges via an open hash
		table keyed by (min,max) endpoint pair. */
	int hashCap = 16;
	while (hashCap < oldTris * 8) hashCap *= 2;
	struct { uint64_t key; int tri; int side; } *eh =
		(void *)calloc((size_t)hashCap, sizeof(*eh));
	for (int i = 0; i < hashCap; ++i) eh[i].key = (uint64_t)-1;

#define EDGE_KEY(x, y) (((uint64_t)(uint32_t)((x) < (y) ? (x) : (y)) << 32) | (uint32_t)((x) > (y) ? (x) : (y)))

	for (int t = 0; t < oldTris; ++t) {
		if (!triAlive[t]) continue;
		int idx[3] = {triV[t * 3 + 0], triV[t * 3 + 1], triV[t * 3 + 2]};
		for (int k = 0; k < 3; ++k) {
			int a = idx[k], b = idx[(k + 1) % 3];
			uint64_t key = EDGE_KEY(a, b);
			uint64_t mask = (uint64_t)(hashCap - 1);
			uint64_t h = (key ^ (key >> 33)) & mask;
			for (;;) {
				if (eh[h].key == (uint64_t)-1) {
					eh[h].key = key; eh[h].tri = t; eh[h].side = 1;
					break;
				}
				if (eh[h].key == key) { eh[h].side++; break; }
				h = (h + 1) & mask;
			}
		}
	}
	for (int i = 0; i < hashCap; ++i) {
		if (eh[i].key == (uint64_t)-1 || eh[i].side != 1) continue;
		int a = (int)(eh[i].key >> 32);
		int b = (int)(eh[i].key & 0xffffffffu);
		int t = eh[i].tri;
		const float *pa = &frame0[a * 3], *pb = &frame0[b * 3];
		int c = triV[t * 3 + 0];
		if (c == a || c == b) c = triV[t * 3 + 1];
		if (c == a || c == b) c = triV[t * 3 + 2];
		const float *pc = &frame0[c * 3];
		/* triangle normal */
		double ux = pb[0] - pa[0], uy = pb[1] - pa[1], uz = pb[2] - pa[2];
		double vx = pc[0] - pa[0], vy = pc[1] - pa[1], vz = pc[2] - pa[2];
		double nx = uy * vz - uz * vy;
		double ny = uz * vx - ux * vz;
		double nz = ux * vy - uy * vx;
		normalize3((double[]){nx, ny, nz});
		double n[3] = {nx, ny, nz};
		normalize3(n);
		double e[3] = {pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]};
		normalize3(e);
		/* Plane perpendicular to face that contains the edge: normal = n x e. */
		double pn[3] = {n[1] * e[2] - n[2] * e[1],
						n[2] * e[0] - n[0] * e[2],
						n[0] * e[1] - n[1] * e[0]};
		normalize3(pn);
		double pd = -(pn[0] * pa[0] + pn[1] * pa[1] + pn[2] * pa[2]);
		const double bndWeight = 1024.0;
		qem_t plane;
		qem_zero(&plane);
		qem_add_plane(&plane, pn[0], pn[1], pn[2], pd, bndWeight);
		qem_add(&Q[a], &plane);
		qem_add(&Q[b], &plane);
	}
	free(eh);

	/* --- Seed the heap with one entry per unique edge --------------- */
	heap_t heap;
	heap_init(&heap, oldTris * 3);

	/* Walk every alive triangle, push each edge once.  Duplicates are
	   harmless: lazy popping recomputes the cost and skips stale
	   entries. */
	for (int t = 0; t < oldTris; ++t) {
		if (!triAlive[t]) continue;
		int idx[3] = {triV[t * 3 + 0], triV[t * 3 + 1], triV[t * 3 + 2]};
		for (int k = 0; k < 3; ++k) {
			int a = idx[k], b = idx[(k + 1) % 3];
			if (a >= b) continue; /* push only canonical orientation */
			qem_t sum = Q[a]; qem_add(&sum, &Q[b]);
			double tParam, cost;
			cost = edge_optimum(&sum, &xyz[a * 3], &xyz[b * 3], &tParam);
			heap_push(&heap, cost, a, b);
		}
	}

	/* --- Greedy collapse loop ------------------------------------- */
	int aliveTris = oldTris;
	for (int t = 0; t < oldTris; ++t) if (!triAlive[t]) aliveTris--;
	const double EPS_COST = 1e-9;

	while (aliveTris > targetTris) {
		heap_entry_t e;
		if (!heap_pop(&heap, &e)) break;
		int v0 = e.v0, v1 = e.v1;
		if (!vertAlive[v0] || !vertAlive[v1]) continue;
		/* Confirm the edge still exists by checking shared triangles. */
		int shared = 0;
		for (int i = 0; i < vertTris[v0].count; ++i) {
			int t = vertTris[v0].items[i];
			if (!triAlive[t]) continue;
			if (iv_contains(&vertTris[v1], t)) { shared = 1; break; }
		}
		if (!shared) continue;
		/* Recompute the cost from current quadrics; if it moved, re-push and
		   continue popping. */
		qem_t sum = Q[v0]; qem_add(&sum, &Q[v1]);
		double tParam, cost;
		cost = edge_optimum(&sum, &xyz[v0 * 3], &xyz[v1 * 3], &tParam);
		if (cost > e.cost + EPS_COST) {
			heap_push(&heap, cost, v0, v1);
			continue;
		}

		/* --- Perform the collapse: v1 -> v0 ------------------------ */
		double tt = tParam;
		double inv_t = 1.0 - tt;
		for (int f = 0; f < numFrames; ++f) {
			float *p0 = &xyz[(size_t)f * oldVerts * 3 + v0 * 3];
			float *p1 = &xyz[(size_t)f * oldVerts * 3 + v1 * 3];
			p0[0] = (float)(inv_t * p0[0] + tt * p1[0]);
			p0[1] = (float)(inv_t * p0[1] + tt * p1[1]);
			p0[2] = (float)(inv_t * p0[2] + tt * p1[2]);
			if (nrm) {
				float *n0 = &nrm[(size_t)f * oldVerts * 3 + v0 * 3];
				float *n1 = &nrm[(size_t)f * oldVerts * 3 + v1 * 3];
				n0[0] = (float)(inv_t * n0[0] + tt * n1[0]);
				n0[1] = (float)(inv_t * n0[1] + tt * n1[1]);
				n0[2] = (float)(inv_t * n0[2] + tt * n1[2]);
				double l = sqrt((double)n0[0] * n0[0] + (double)n0[1] * n0[1] + (double)n0[2] * n0[2]);
				if (l > 1e-20) { n0[0] = (float)(n0[0] / l); n0[1] = (float)(n0[1] / l); n0[2] = (float)(n0[2] / l); }
			}
		}
		if (st) {
			float *u0 = &st[v0 * 2], *u1 = &st[v1 * 2];
			u0[0] = (float)(inv_t * u0[0] + tt * u1[0]);
			u0[1] = (float)(inv_t * u0[1] + tt * u1[1]);
		}
		qem_add(&Q[v0], &Q[v1]);

		/* Walk v1's triangles: if also incident to v0, kill it
		   (degenerate after merge); else replace v1 with v0 and add
		   the triangle to v0's list. */
		for (int i = 0; i < vertTris[v1].count; ++i) {
			int t = vertTris[v1].items[i];
			if (!triAlive[t]) continue;
			int *vs = &triV[t * 3];
			if (vs[0] == v0 || vs[1] == v0 || vs[2] == v0) {
				triAlive[t] = 0;
				aliveTris--;
				/* Remove from the third vertex's list. */
				int other = -1;
				for (int k = 0; k < 3; ++k) {
					if (vs[k] != v0 && vs[k] != v1) { other = vs[k]; break; }
				}
				if (other >= 0) iv_remove(&vertTris[other], t);
				/* v0's tri list is updated below by removing t from v0's list. */
				iv_remove(&vertTris[v0], t);
			} else {
				for (int k = 0; k < 3; ++k) if (vs[k] == v1) vs[k] = v0;
				iv_push(&vertTris[v0], t);
			}
		}
		vertTris[v1].count = 0;
		vertAlive[v1] = 0;
		vertParent[v1] = v0;

		/* Re-queue all edges around v0 with fresh costs. */
		for (int i = 0; i < vertTris[v0].count; ++i) {
			int t = vertTris[v0].items[i];
			if (!triAlive[t]) continue;
			int *vs = &triV[t * 3];
			for (int k = 0; k < 3; ++k) {
				int u = vs[k];
				if (u == v0) continue;
				int a = v0 < u ? v0 : u;
				int b = v0 < u ? u : v0;
				qem_t sum2 = Q[a]; qem_add(&sum2, &Q[b]);
				double tp, cc;
				cc = edge_optimum(&sum2, &xyz[a * 3], &xyz[b * 3], &tp);
				heap_push(&heap, cc, a, b);
			}
		}
	}
	heap_free(&heap);

	/* --- Compaction: remap surviving vertices and rebuild arrays --- */
	int *newIdxOf = (int *)malloc(sizeof(int) * (size_t)oldVerts);
	int newVerts = 0;
	for (int i = 0; i < oldVerts; ++i) {
		newIdxOf[i] = vertAlive[i] ? newVerts++ : -1;
	}
	/* Resolve dead vertices to their final survivor (path compression). */
	for (int i = 0; i < oldVerts; ++i) {
		if (vertAlive[i]) continue;
		int p = i;
		while (p >= 0 && !vertAlive[p]) p = vertParent[p];
		newIdxOf[i] = (p >= 0) ? newIdxOf[p] : -1;
	}

	int newTris = 0;
	for (int t = 0; t < oldTris; ++t) if (triAlive[t]) newTris++;

	float *outXYZ = (float *)malloc(sizeof(float) * (size_t)numFrames * (size_t)newVerts * 3);
	float *outST = (float *)malloc(sizeof(float) * 2 * (size_t)newVerts);
	int *outIdx = (int *)malloc(sizeof(int) * (size_t)newTris * 3);

	for (int i = 0; i < oldVerts; ++i) {
		if (!vertAlive[i]) continue;
		int ni = newIdxOf[i];
		for (int f = 0; f < numFrames; ++f) {
			float *src = &xyz[(size_t)f * oldVerts * 3 + i * 3];
			float *dst = &outXYZ[(size_t)f * newVerts * 3 + ni * 3];
			dst[0] = src[0]; dst[1] = src[1]; dst[2] = src[2];
		}
		if (st) {
			outST[ni * 2 + 0] = st[i * 2 + 0];
			outST[ni * 2 + 1] = st[i * 2 + 1];
		} else {
			outST[ni * 2 + 0] = 0.0f;
			outST[ni * 2 + 1] = 0.0f;
		}
	}

	int wt = 0;
	for (int t = 0; t < oldTris; ++t) {
		if (!triAlive[t]) continue;
		int a = newIdxOf[triV[t * 3 + 0]];
		int b = newIdxOf[triV[t * 3 + 1]];
		int c = newIdxOf[triV[t * 3 + 2]];
		outIdx[wt * 3 + 0] = a;
		outIdx[wt * 3 + 1] = b;
		outIdx[wt * 3 + 2] = c;
		wt++;
	}

	/* Propagate per-frame normals from the (collapsed-and-blended)
	   working buffer.  We deliberately do NOT recompute from
	   geometric face cross products: MD3 winding can be opposite to
	   the author-controlled per-vertex normals, so recomputed normals
	   would point the wrong way and make the texture appear on the
	   back side. */
	float *outN = (float *)calloc((size_t)numFrames * (size_t)newVerts * 3, sizeof(float));
	if (nrm) {
		for (int i = 0; i < oldVerts; ++i) {
			if (!vertAlive[i]) continue;
			int ni = newIdxOf[i];
			for (int f = 0; f < numFrames; ++f) {
				const float *src = &nrm[(size_t)f * oldVerts * 3 + i * 3];
				float *dst = &outN[(size_t)f * newVerts * 3 + ni * 3];
				dst[0] = src[0]; dst[1] = src[1]; dst[2] = src[2];
			}
		}
	} else {
		/* No source normals: fall back to geometric face normals. */
		for (int f = 0; f < numFrames; ++f) {
			const float *fxyz = &outXYZ[(size_t)f * newVerts * 3];
			float *fn = &outN[(size_t)f * newVerts * 3];
			for (int t = 0; t < newTris; ++t) {
				int ia = outIdx[t * 3 + 0], ib = outIdx[t * 3 + 1], ic = outIdx[t * 3 + 2];
				const float *p0 = &fxyz[ia * 3], *p1 = &fxyz[ib * 3], *p2 = &fxyz[ic * 3];
				double ux = p1[0] - p0[0], uy = p1[1] - p0[1], uz = p1[2] - p0[2];
				double vx = p2[0] - p0[0], vy = p2[1] - p0[1], vz = p2[2] - p0[2];
				float n[3] = {(float)(uy * vz - uz * vy),
							  (float)(uz * vx - ux * vz),
							  (float)(ux * vy - uy * vx)};
				for (int k = 0; k < 3; ++k) {
					fn[ia * 3 + k] += n[k];
					fn[ib * 3 + k] += n[k];
					fn[ic * 3 + k] += n[k];
				}
			}
			for (int v = 0; v < newVerts; ++v) {
				float *n = &fn[v * 3];
				float l = sqrtf(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
				if (l > 1e-20f) { n[0] /= l; n[1] /= l; n[2] /= l; }
			}
		}
	}

	/* --- Swap into surface ---------------------------------------- */
	free(s->xyz); free(s->normal); free(s->st); free(s->indices);
	s->xyz = outXYZ;
	s->normal = outN;
	s->st = outST;
	s->indices = outIdx;
	s->numVerts = newVerts;
	s->numTris = newTris;

	/* --- Cleanup -------------------------------------------------- */
	free(Q);
	for (int i = 0; i < oldVerts; ++i) free(vertTris[i].items);
	free(vertTris);
	free(triAlive); free(triV);
	free(vertAlive); free(vertParent);
	free(xyz); free(nrm); free(st);
	free(newIdxOf);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Public entry point                                                 */
/* ------------------------------------------------------------------ */

int mc_decimate(mc_model_t *m, float ratio) {
	if (!m || ratio >= 1.0f) return 0;
	if (ratio < 0.0f) ratio = 0.0f;
	for (int i = 0; i < m->numSurfaces; ++i) {
		mc_surface_t *s = &m->surfaces[i];
		if (s->deform[0]) {
			/* autosprite / autosprite2 require the original quad layout;
			   decimating would break the camera-facing math. Pass through
			   unchanged - this is the correct action, not an error. */
			MC_DEBUG("decimate: surface %d '%s' shader='%s' kept as-is (deformVertexes %s requires quad layout)\n",
					 i, s->name, s->shader, s->deform);
			continue;
		}
		int oldT = s->numTris, oldV = s->numVerts;
		int targetT = (int)floorf((float)oldT * ratio + 0.5f);
		if (targetT < 1) targetT = 1;
		if (targetT >= oldT) continue;
		if (decimate_surface(s, m->numFrames, targetT) != 0)
			return -1;
		MC_LOG("decimate: surface %d '%s' tris %d->%d (verts %d->%d)\n",
			   i, s->name, oldT, s->numTris, oldV, s->numVerts);
	}
	/* Refresh per-frame bounds. */
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
	return 0;
}

/* ------------------------------------------------------------------ */
/* LOD generator                                                      */
/* ------------------------------------------------------------------ */

int mc_gen_lods(mc_model_t *m, int numLods, const float *ratios) {
	if (!m || numLods <= 0 || !ratios) return 0;
	if (numLods > MD3_MAX_LODS - 1) numLods = MD3_MAX_LODS - 1;

	/* Snapshot indices of base LOD0 surfaces so newly appended LOD>0
	   copies don't enter the iteration. */
	int *baseIdx = (int *)calloc((size_t)m->numSurfaces, sizeof(int));
	int baseCount = 0;
	for (int i = 0; i < m->numSurfaces; ++i) {
		if (m->surfaces[i].lod == 0) baseIdx[baseCount++] = i;
	}
	if (baseCount == 0) { free(baseIdx); return 0; }

	for (int L = 1; L <= numLods; ++L) {
		float r = ratios[L - 1];
		if (r <= 0.0f) r = 0.01f;
		if (r > 1.0f)  r = 1.0f;
		for (int bi = 0; bi < baseCount; ++bi) {
			/* mc_model_add_surface may realloc, so capture src by index, then refresh ptr. */
			int srcIdx = baseIdx[bi];
			mc_surface_t *dst = mc_model_add_surface(m);
			if (!dst) { free(baseIdx); return -1; }
			const mc_surface_t *src = &m->surfaces[srcIdx];

			mc_q_strncpy(dst->name, src->name, sizeof(dst->name));
			mc_q_strncpy(dst->shader, src->shader, sizeof(dst->shader));
			mc_q_strncpy(dst->texture, src->texture, sizeof(dst->texture));
			mc_q_strncpy(dst->part, src->part, sizeof(dst->part));
			mc_q_strncpy(dst->deform, src->deform, sizeof(dst->deform));
			mc_q_strncpy(dst->normal_map, src->normal_map, sizeof(dst->normal_map));
			mc_q_strncpy(dst->normal_height_map, src->normal_height_map, sizeof(dst->normal_height_map));
			mc_q_strncpy(dst->emissive_map, src->emissive_map, sizeof(dst->emissive_map));
			mc_q_strncpy(dst->mr_map, src->mr_map, sizeof(dst->mr_map));
			mc_q_strncpy(dst->surfaceparm, src->surfaceparm, sizeof(dst->surfaceparm));
			memcpy(dst->base_color, src->base_color, sizeof(dst->base_color));
			memcpy(dst->emissive_factor, src->emissive_factor, sizeof(dst->emissive_factor));
			dst->two_sided = src->two_sided;
			dst->alpha_mode = src->alpha_mode;
			dst->alpha_cutoff = src->alpha_cutoff;
			dst->lod = L;

			int frames = m->numFrames > 0 ? m->numFrames : 1;
			mc_surface_alloc(dst, src->numVerts, src->numTris, frames);
			memcpy(dst->xyz, src->xyz, (size_t)src->numVerts * (size_t)frames * 3 * sizeof(float));
			memcpy(dst->normal, src->normal, (size_t)src->numVerts * (size_t)frames * 3 * sizeof(float));
			if (src->st)
				memcpy(dst->st, src->st, (size_t)src->numVerts * 2 * sizeof(float));
			memcpy(dst->indices, src->indices, (size_t)src->numTris * 3 * sizeof(int));

			if (dst->deform[0]) {
				MC_LOG("gen-lods: lod %d surface '%s' kept full detail (deform '%s' locks topology)\n",
					   L, dst->name, dst->deform);
				continue;
			}
			int targetT = (int)floorf((float)dst->numTris * r + 0.5f);
			if (targetT < 1) targetT = 1;
			if (targetT >= dst->numTris) {
				MC_LOG("gen-lods: lod %d surface '%s' tris %d (no reduction at ratio %.3f)\n",
					   L, dst->name, dst->numTris, r);
				continue;
			}
			int oldT = dst->numTris, oldV = dst->numVerts;
			if (decimate_surface(dst, frames, targetT) != 0) {
				free(baseIdx);
				return -1;
			}
			MC_LOG("gen-lods: lod %d surface '%s' tris %d->%d (verts %d->%d)\n",
				   L, dst->name, oldT, dst->numTris, oldV, dst->numVerts);
		}
	}

	free(baseIdx);
	return 0;
}