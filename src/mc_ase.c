/*
===========================================================================
modelconverter - ASE (3D Studio ASCII Export) reader/writer

Reads and writes the subset of ASE relevant to Quake 3 model pipelines:
  - GEOMOBJECT meshes (vertices, faces, texture coords, normals)
  - MATERIAL_LIST (bitmap paths become surface shader names)
  - Nodes named "tag_*" are treated as tags (origin from first vertex)

Based on q3map2 ase parser
===========================================================================
*/

#include "mc_common.h"
#include <ctype.h>

/* ---- Tokenizer ---- */

typedef struct {
	const char *buf;
	size_t len;
	size_t pos;
	char token[1024];
} ase_parser_t;

static int ase_get_token(ase_parser_t *p, int rest_of_line) {
	/* skip whitespace */
	while (p->pos < p->len && (unsigned char)p->buf[p->pos] <= 32)
		p->pos++;
	if (p->pos >= p->len)
		return 0;
	int i = 0;
	while (p->pos < p->len && i < 1023) {
		char c = p->buf[p->pos];
		p->token[i] = c;
		p->pos++;
		i++;
		if (!rest_of_line && (unsigned char)c <= 32) { i--; break; }
		if (c == '\n' || c == '\r') { i--; break; }
	}
	p->token[i] = 0;
	return 1;
}

static void ase_skip_rest_of_line(ase_parser_t *p) {
	ase_get_token(p, 1);
}

static void ase_skip_block(ase_parser_t *p) {
	int depth = 0;
	while (ase_get_token(p, 0)) {
		if (!strcmp(p->token, "{")) depth++;
		else if (!strcmp(p->token, "}")) { if (--depth <= 0) break; }
	}
}

/* ---- ASE intermediate structures ---- */

#define ASE_MAX_MATERIALS 64
#define ASE_MAX_OBJECTS   128

typedef struct {
	char name[MC_MAX_PATH];
} ase_material_t;

typedef struct {
	float x, y, z;
} ase_vertex_t;

typedef struct {
	float s, t;
} ase_tvertex_t;

typedef struct {
	int v[3];
} ase_face_t;

typedef struct {
	char name[128];
	int materialRef;
	int numVerts;
	int numFaces;
	int numTVerts;
	ase_vertex_t *verts;
	ase_tvertex_t *tverts;
	ase_face_t *faces;
	ase_face_t *tfaces;
} ase_object_t;

typedef struct {
	ase_material_t materials[ASE_MAX_MATERIALS];
	int numMaterials;
	ase_object_t objects[ASE_MAX_OBJECTS];
	int numObjects;
} ase_scene_t;

static void ase_free_scene(ase_scene_t *s) {
	for (int i = 0; i < s->numObjects; i++) {
		free(s->objects[i].verts);
		free(s->objects[i].tverts);
		free(s->objects[i].faces);
		free(s->objects[i].tfaces);
	}
}

/* ---- Parse helpers ---- */

static void ase_parse_mesh_vertex_list(ase_parser_t *p, ase_object_t *obj) {
	int depth = 0;
	while (ase_get_token(p, 0)) {
		if (!strcmp(p->token, "{")) { depth++; continue; }
		if (!strcmp(p->token, "}")) { if (--depth <= 0) break; continue; }
		if (!strcmp(p->token, "*MESH_VERTEX")) {
			ase_get_token(p, 0); /* index */
			int idx = atoi(p->token);
			if (idx >= 0 && idx < obj->numVerts) {
				ase_get_token(p, 0); obj->verts[idx].x = (float)atof(p->token);
				ase_get_token(p, 0); obj->verts[idx].y = (float)atof(p->token);
				ase_get_token(p, 0); obj->verts[idx].z = (float)atof(p->token);
			}
		}
	}
}

static void ase_parse_mesh_face_list(ase_parser_t *p, ase_object_t *obj) {
	int depth = 0, fi = 0;
	while (ase_get_token(p, 0)) {
		if (!strcmp(p->token, "{")) { depth++; continue; }
		if (!strcmp(p->token, "}")) { if (--depth <= 0) break; continue; }
		if (!strcmp(p->token, "*MESH_FACE")) {
			if (fi >= obj->numFaces) { ase_skip_rest_of_line(p); continue; }
			ase_get_token(p, 0); /* face number: */
			ase_get_token(p, 0); /* A: */
			ase_get_token(p, 0); obj->faces[fi].v[0] = atoi(p->token);
			ase_get_token(p, 0); /* B: */
			ase_get_token(p, 0); obj->faces[fi].v[1] = atoi(p->token);
			ase_get_token(p, 0); /* C: */
			ase_get_token(p, 0); obj->faces[fi].v[2] = atoi(p->token);
			ase_skip_rest_of_line(p);
			fi++;
		}
	}
}

static void ase_parse_mesh_tvertlist(ase_parser_t *p, ase_object_t *obj) {
	int depth = 0;
	while (ase_get_token(p, 0)) {
		if (!strcmp(p->token, "{")) { depth++; continue; }
		if (!strcmp(p->token, "}")) { if (--depth <= 0) break; continue; }
		if (!strcmp(p->token, "*MESH_TVERT")) {
			ase_get_token(p, 0); int idx = atoi(p->token);
			if (idx >= 0 && idx < obj->numTVerts) {
				ase_get_token(p, 0); obj->tverts[idx].s = (float)atof(p->token);
				ase_get_token(p, 0); obj->tverts[idx].t = 1.0f - (float)atof(p->token);
				ase_get_token(p, 0); /* w - ignored */
			}
		}
	}
}

static void ase_parse_mesh_tfacelist(ase_parser_t *p, ase_object_t *obj) {
	int depth = 0, fi = 0;
	while (ase_get_token(p, 0)) {
		if (!strcmp(p->token, "{")) { depth++; continue; }
		if (!strcmp(p->token, "}")) { if (--depth <= 0) break; continue; }
		if (!strcmp(p->token, "*MESH_TFACE")) {
			if (fi >= obj->numFaces) { ase_skip_rest_of_line(p); fi++; continue; }
			ase_get_token(p, 0); /* face index */
			ase_get_token(p, 0); obj->tfaces[fi].v[0] = atoi(p->token);
			ase_get_token(p, 0); obj->tfaces[fi].v[1] = atoi(p->token);
			ase_get_token(p, 0); obj->tfaces[fi].v[2] = atoi(p->token);
			fi++;
		}
	}
}

static void ase_parse_mesh(ase_parser_t *p, ase_object_t *obj) {
	int depth = 0;
	while (ase_get_token(p, 0)) {
		if (!strcmp(p->token, "{")) { depth++; continue; }
		if (!strcmp(p->token, "}")) { if (--depth <= 0) break; continue; }
		if (!strcmp(p->token, "*MESH_NUMVERTEX")) {
			ase_get_token(p, 0);
			obj->numVerts = atoi(p->token);
		} else if (!strcmp(p->token, "*MESH_NUMFACES")) {
			ase_get_token(p, 0);
			obj->numFaces = atoi(p->token);
		} else if (!strcmp(p->token, "*MESH_NUMTVERTEX")) {
			ase_get_token(p, 0);
			obj->numTVerts = atoi(p->token);
		} else if (!strcmp(p->token, "*MESH_VERTEX_LIST")) {
			if (!obj->verts && obj->numVerts > 0)
				obj->verts = (ase_vertex_t *)mc_calloc((size_t)obj->numVerts, sizeof(ase_vertex_t));
			ase_parse_mesh_vertex_list(p, obj);
		} else if (!strcmp(p->token, "*MESH_FACE_LIST")) {
			if (!obj->faces && obj->numFaces > 0)
				obj->faces = (ase_face_t *)mc_calloc((size_t)obj->numFaces, sizeof(ase_face_t));
			ase_parse_mesh_face_list(p, obj);
		} else if (!strcmp(p->token, "*MESH_TVERTLIST")) {
			if (!obj->tverts && obj->numTVerts > 0)
				obj->tverts = (ase_tvertex_t *)mc_calloc((size_t)obj->numTVerts, sizeof(ase_tvertex_t));
			ase_parse_mesh_tvertlist(p, obj);
		} else if (!strcmp(p->token, "*MESH_TFACELIST")) {
			if (!obj->tfaces && obj->numFaces > 0)
				obj->tfaces = (ase_face_t *)mc_calloc((size_t)obj->numFaces, sizeof(ase_face_t));
			ase_parse_mesh_tfacelist(p, obj);
		} else if (!strcmp(p->token, "*MESH_NORMALS")) {
			ase_skip_block(p);
		}
	}
}

static void ase_parse_geomobject(ase_parser_t *p, ase_scene_t *scene) {
	if (scene->numObjects >= ASE_MAX_OBJECTS) { ase_skip_block(p); return; }
	ase_object_t *obj = &scene->objects[scene->numObjects];
	memset(obj, 0, sizeof(*obj));
	int depth = 0;
	while (ase_get_token(p, 0)) {
		if (!strcmp(p->token, "{")) { depth++; continue; }
		if (!strcmp(p->token, "}")) { if (--depth <= 0) break; continue; }
		if (!strcmp(p->token, "*NODE_NAME")) {
			ase_get_token(p, 1);
			/* strip quotes */
			char *s = p->token;
			if (*s == '"') s++;
			mc_q_strncpy(obj->name, s, sizeof(obj->name));
			char *q = strchr(obj->name, '"');
			if (q) *q = 0;
		} else if (!strcmp(p->token, "*MESH")) {
			ase_parse_mesh(p, obj);
		} else if (!strcmp(p->token, "*MATERIAL_REF")) {
			ase_get_token(p, 0);
			obj->materialRef = atoi(p->token);
		} else if (!strcmp(p->token, "*NODE_TM") ||
		           !strcmp(p->token, "*TM_ANIMATION") ||
		           !strcmp(p->token, "*MESH_ANIMATION")) {
			ase_skip_block(p);
		} else if (!strcmp(p->token, "*PROP_MOTIONBLUR") ||
		           !strcmp(p->token, "*PROP_CASTSHADOW") ||
		           !strcmp(p->token, "*PROP_RECVSHADOW") ||
		           !strcmp(p->token, "*NODE_PARENT")) {
			ase_skip_rest_of_line(p);
		}
	}
	scene->numObjects++;
}

static void ase_parse_material(ase_parser_t *p, ase_scene_t *scene) {
	int idx = scene->numMaterials;
	if (idx >= ASE_MAX_MATERIALS) { ase_skip_block(p); return; }
	int depth = 0;
	while (ase_get_token(p, 0)) {
		if (!strcmp(p->token, "{")) { depth++; continue; }
		if (!strcmp(p->token, "}")) { if (--depth <= 0) break; continue; }
		if (!strcmp(p->token, "*BITMAP")) {
			ase_get_token(p, 0);
			char *s = p->token;
			if (*s == '"') s++;
			mc_q_strncpy(scene->materials[idx].name, s, sizeof(scene->materials[idx].name));
			char *q = strchr(scene->materials[idx].name, '"');
			if (q) *q = 0;
			/* convert backslashes */
			for (char *c = scene->materials[idx].name; *c; c++)
				if (*c == '\\') *c = '/';
		} else if (!strcmp(p->token, "*MAP_DIFFUSE") ||
		           !strcmp(p->token, "*MAP_BUMP") ||
		           !strcmp(p->token, "*MAP_SPECULAR")) {
			/* recurse into sub-block to find *BITMAP */
			int sd = 0;
			while (ase_get_token(p, 0)) {
				if (!strcmp(p->token, "{")) { sd++; continue; }
				if (!strcmp(p->token, "}")) { if (--sd <= 0) break; continue; }
				if (!strcmp(p->token, "*BITMAP") && !scene->materials[idx].name[0]) {
					ase_get_token(p, 0);
					char *bs = p->token;
					if (*bs == '"') bs++;
					mc_q_strncpy(scene->materials[idx].name, bs, sizeof(scene->materials[idx].name));
					char *bq = strchr(scene->materials[idx].name, '"');
					if (bq) *bq = 0;
					for (char *c = scene->materials[idx].name; *c; c++)
						if (*c == '\\') *c = '/';
				}
			}
		}
	}
	scene->numMaterials++;
}

static void ase_parse_material_list(ase_parser_t *p, ase_scene_t *scene) {
	int depth = 0;
	while (ase_get_token(p, 0)) {
		if (!strcmp(p->token, "{")) { depth++; continue; }
		if (!strcmp(p->token, "}")) { if (--depth <= 0) break; continue; }
		if (!strcmp(p->token, "*MATERIAL")) {
			ase_get_token(p, 0); /* material index */
			ase_parse_material(p, scene);
		}
	}
}

/* ---- Build mc_model_t from parsed ASE scene ---- */

static void ase_compute_normal(const float *v0, const float *v1, const float *v2, float *out) {
	float e1[3] = { v1[0]-v0[0], v1[1]-v0[1], v1[2]-v0[2] };
	float e2[3] = { v2[0]-v0[0], v2[1]-v0[1], v2[2]-v0[2] };
	out[0] = e1[1]*e2[2] - e1[2]*e2[1];
	out[1] = e1[2]*e2[0] - e1[0]*e2[2];
	out[2] = e1[0]*e2[1] - e1[1]*e2[0];
	float len = sqrtf(out[0]*out[0] + out[1]*out[1] + out[2]*out[2]);
	if (len > 1e-6f) { out[0]/=len; out[1]/=len; out[2]/=len; }
	else { out[0]=0; out[1]=0; out[2]=1.0f; }
}

static int ase_is_tag(const char *name) {
	return (strncasecmp(name, "tag_", 4) == 0 || strncasecmp(name, "tag ", 4) == 0);
}

static int ase_build_model(const ase_scene_t *scene, mc_model_t *out) {
	mc_model_init(out);

	/* Single frame for static ASE models */
	mc_frame_t *fr = mc_model_add_frame(out);
	mc_q_strncpy(fr->name, "frame0", sizeof(fr->name));

	for (int oi = 0; oi < scene->numObjects; oi++) {
		const ase_object_t *obj = &scene->objects[oi];

		/* Tags: nodes named tag_* with geometry - use centroid as origin */
		if (ase_is_tag(obj->name)) {
			mc_tag_t *tag = mc_model_add_tag_slot(out, out->numFrames);
			mc_q_strncpy(tag->name, obj->name, sizeof(tag->name));
			/* compute centroid from vertices */
			if (obj->numVerts > 0 && obj->verts) {
				float cx = 0, cy = 0, cz = 0;
				for (int v = 0; v < obj->numVerts; v++) {
					cx += obj->verts[v].x;
					cy += obj->verts[v].y;
					cz += obj->verts[v].z;
				}
				tag->origin[0] = cx / obj->numVerts;
				tag->origin[1] = cy / obj->numVerts;
				tag->origin[2] = cz / obj->numVerts;
			}
			/* identity axes */
			tag->axis[0][0] = 1; tag->axis[1][1] = 1; tag->axis[2][2] = 1;
			continue;
		}

		if (obj->numVerts <= 0 || obj->numFaces <= 0 || !obj->verts || !obj->faces)
			continue;

		/* Expand indexed mesh into per-face-vertex (unique pos+st combos) */
		int numTris = obj->numFaces;
		int numOutVerts = numTris * 3;

		mc_surface_t *surf = mc_model_add_surface(out);
		mc_q_strncpy(surf->name, obj->name, sizeof(surf->name));

		/* Set shader from material */
		if (obj->materialRef >= 0 && obj->materialRef < scene->numMaterials) {
			const char *matName = scene->materials[obj->materialRef].name;
			mc_q_strncpy(surf->shader, matName, sizeof(surf->shader));
			mc_q_strncpy(surf->texture, matName, sizeof(surf->texture));
		}

		mc_surface_alloc(surf, numOutVerts, numTris, 1);

		for (int fi = 0; fi < numTris; fi++) {
			int i0 = obj->faces[fi].v[0];
			int i1 = obj->faces[fi].v[1];
			int i2 = obj->faces[fi].v[2];

			/* clamp indices */
			if (i0 < 0 || i0 >= obj->numVerts) i0 = 0;
			if (i1 < 0 || i1 >= obj->numVerts) i1 = 0;
			if (i2 < 0 || i2 >= obj->numVerts) i2 = 0;

			int base = fi * 3;
			surf->xyz[base*3+0] = obj->verts[i0].x;
			surf->xyz[base*3+1] = obj->verts[i0].y;
			surf->xyz[base*3+2] = obj->verts[i0].z;
			surf->xyz[(base+1)*3+0] = obj->verts[i1].x;
			surf->xyz[(base+1)*3+1] = obj->verts[i1].y;
			surf->xyz[(base+1)*3+2] = obj->verts[i1].z;
			surf->xyz[(base+2)*3+0] = obj->verts[i2].x;
			surf->xyz[(base+2)*3+1] = obj->verts[i2].y;
			surf->xyz[(base+2)*3+2] = obj->verts[i2].z;

			/* texture coords */
			if (obj->tverts && obj->tfaces) {
				int t0 = obj->tfaces[fi].v[0];
				int t1 = obj->tfaces[fi].v[1];
				int t2 = obj->tfaces[fi].v[2];
				if (t0 >= 0 && t0 < obj->numTVerts) {
					surf->st[base*2+0] = obj->tverts[t0].s;
					surf->st[base*2+1] = obj->tverts[t0].t;
				}
				if (t1 >= 0 && t1 < obj->numTVerts) {
					surf->st[(base+1)*2+0] = obj->tverts[t1].s;
					surf->st[(base+1)*2+1] = obj->tverts[t1].t;
				}
				if (t2 >= 0 && t2 < obj->numTVerts) {
					surf->st[(base+2)*2+0] = obj->tverts[t2].s;
					surf->st[(base+2)*2+1] = obj->tverts[t2].t;
				}
			}

			/* face normal for all 3 verts */
			float n[3];
			ase_compute_normal(&surf->xyz[base*3], &surf->xyz[(base+1)*3], &surf->xyz[(base+2)*3], n);
			for (int k = 0; k < 3; k++) {
				surf->normal[(base+k)*3+0] = n[0];
				surf->normal[(base+k)*3+1] = n[1];
				surf->normal[(base+k)*3+2] = n[2];
			}

			surf->indices[fi*3+0] = base;
			surf->indices[fi*3+1] = base+1;
			surf->indices[fi*3+2] = base+2;
		}
	}

	/* Compute frame bounds */
	if (out->numSurfaces > 0 && out->numFrames > 0) {
		mc_frame_t *f = &out->frames[0];
		float mins[3] = {1e30f,1e30f,1e30f}, maxs[3] = {-1e30f,-1e30f,-1e30f};
		for (int si = 0; si < out->numSurfaces; si++) {
			const mc_surface_t *s = &out->surfaces[si];
			for (int v = 0; v < s->numVerts; v++) {
				for (int k = 0; k < 3; k++) {
					if (s->xyz[v*3+k] < mins[k]) mins[k] = s->xyz[v*3+k];
					if (s->xyz[v*3+k] > maxs[k]) maxs[k] = s->xyz[v*3+k];
				}
			}
		}
		for (int k = 0; k < 3; k++) {
			f->bounds[0][k] = mins[k];
			f->bounds[1][k] = maxs[k];
			f->localOrigin[k] = 0.5f * (mins[k] + maxs[k]);
		}
		float dx = maxs[0]-mins[0], dy = maxs[1]-mins[1], dz = maxs[2]-mins[2];
		f->radius = 0.5f * sqrtf(dx*dx + dy*dy + dz*dz);
	}

	return 0;
}

/* ---- Public API: mc_load_ase ---- */

int mc_load_ase(const char *path, mc_model_t *out) {
	size_t size = 0;
	unsigned char *buf = mc_read_file(path, &size);
	if (!buf) return -1;

	ase_parser_t parser;
	parser.buf = (const char *)buf;
	parser.len = size;
	parser.pos = 0;

	ase_scene_t scene;
	memset(&scene, 0, sizeof(scene));

	while (ase_get_token(&parser, 0)) {
		if (!strcmp(parser.token, "*3DSMAX_ASCIIEXPORT") ||
		    !strcmp(parser.token, "*COMMENT")) {
			ase_skip_rest_of_line(&parser);
		} else if (!strcmp(parser.token, "*SCENE")) {
			ase_skip_block(&parser);
		} else if (!strcmp(parser.token, "*MATERIAL_LIST")) {
			ase_parse_material_list(&parser, &scene);
		} else if (!strcmp(parser.token, "*GEOMOBJECT")) {
			ase_parse_geomobject(&parser, &scene);
		}
	}

	int rc = ase_build_model(&scene, out);
	ase_free_scene(&scene);
	free(buf);

	MC_INFO("ase: loaded %s (%d surfaces, %d tags)\n", path, out->numSurfaces, out->numTags);
	return rc;
}

/* ---- Public API: mc_save_ase ---- */

int mc_save_ase(const char *path, const mc_model_t *m) {
	FILE *f = fopen(path, "wb");
	if (!f) {
		MC_ERR("ase: cannot open '%s' for writing\n", path);
		return -1;
	}

	fprintf(f, "*3DSMAX_ASCIIEXPORT\t200\r\n");
	fprintf(f, "*COMMENT\t\"Generated by modelconverter\"\r\n");

	/* Scene block */
	fprintf(f, "*SCENE\t{\r\n");
	fprintf(f, "\t*SCENE_FILENAME\t\"\"\r\n");
	fprintf(f, "\t*SCENE_FIRSTFRAME\t0\r\n");
	fprintf(f, "\t*SCENE_LASTFRAME\t0\r\n");
	fprintf(f, "\t*SCENE_FRAMESPEED\t30\r\n");
	fprintf(f, "\t*SCENE_TICKSPERFRAME\t160\r\n");
	fprintf(f, "}\r\n");

	/* Material list */
	fprintf(f, "*MATERIAL_LIST\t{\r\n");
	fprintf(f, "\t*MATERIAL_COUNT\t%d\r\n", m->numSurfaces);
	for (int i = 0; i < m->numSurfaces; i++) {
		const mc_surface_t *s = &m->surfaces[i];
		const char *shader = s->shader[0] ? s->shader : s->name;
		fprintf(f, "\t*MATERIAL\t%d\t{\r\n", i);
		fprintf(f, "\t\t*MATERIAL_NAME\t\"%s\"\r\n", shader);
		fprintf(f, "\t\t*MATERIAL_CLASS\t\"Standard\"\r\n");
		fprintf(f, "\t\t*MATERIAL_DIFFUSE\t0.5882\t0.5882\t0.5882\r\n");
		fprintf(f, "\t\t*MATERIAL_SHADING Phong\r\n");
		fprintf(f, "\t\t*MAP_DIFFUSE\t{\r\n");
		fprintf(f, "\t\t\t*MAP_NAME\t\"%s\"\r\n", shader);
		fprintf(f, "\t\t\t*MAP_CLASS\t\"Bitmap\"\r\n");
		fprintf(f, "\t\t\t*MAP_SUBNO\t1\r\n");
		fprintf(f, "\t\t\t*MAP_AMOUNT\t1.0\r\n");
		fprintf(f, "\t\t\t*MAP_TYPE\tScreen\r\n");
		fprintf(f, "\t\t\t*BITMAP\t\"%s\"\r\n", shader);
		fprintf(f, "\t\t\t*BITMAP_FILTER\tPyramidal\r\n");
		fprintf(f, "\t\t}\r\n");
		fprintf(f, "\t}\r\n");
	}
	fprintf(f, "}\r\n");

	/* Geometry objects - write frame 0 only */
	for (int si = 0; si < m->numSurfaces; si++) {
		const mc_surface_t *s = &m->surfaces[si];
		const char *name = s->name[0] ? s->name : "surface";

		fprintf(f, "*GEOMOBJECT\t{\r\n");
		fprintf(f, "\t*NODE_NAME\t\"%s\"\r\n", name);
		fprintf(f, "\t*NODE_TM\t{\r\n");
		fprintf(f, "\t\t*NODE_NAME\t\"%s\"\r\n", name);
		fprintf(f, "\t\t*INHERIT_POS\t0\t0\t0\r\n");
		fprintf(f, "\t\t*INHERIT_ROT\t0\t0\t0\r\n");
		fprintf(f, "\t\t*INHERIT_SCL\t0\t0\t0\r\n");
		fprintf(f, "\t\t*TM_ROW0\t1.0\t0\t0\r\n");
		fprintf(f, "\t\t*TM_ROW1\t0\t1.0\t0\r\n");
		fprintf(f, "\t\t*TM_ROW2\t0\t0\t1.0\r\n");
		fprintf(f, "\t\t*TM_ROW3\t0\t0\t0\r\n");
		fprintf(f, "\t\t*TM_POS\t0.0000\t0.0000\t0.0000\r\n");
		fprintf(f, "\t}\r\n");

		fprintf(f, "\t*MESH\t{\r\n");
		fprintf(f, "\t\t*TIMEVALUE\t0\r\n");
		fprintf(f, "\t\t*MESH_NUMVERTEX\t%d\r\n", s->numVerts);
		fprintf(f, "\t\t*MESH_NUMFACES\t%d\r\n", s->numTris);

		/* Vertices */
		fprintf(f, "\t\t*MESH_VERTEX_LIST\t{\r\n");
		for (int v = 0; v < s->numVerts; v++) {
			fprintf(f, "\t\t\t*MESH_VERTEX\t%d\t%f\t%f\t%f\r\n",
				v, s->xyz[v*3+0], s->xyz[v*3+1], s->xyz[v*3+2]);
		}
		fprintf(f, "\t\t}\r\n");

		/* Faces */
		fprintf(f, "\t\t*MESH_FACE_LIST\t{\r\n");
		for (int t = 0; t < s->numTris; t++) {
			int a = s->indices[t*3+0];
			int b = s->indices[t*3+1];
			int c = s->indices[t*3+2];
			fprintf(f, "\t\t\t*MESH_FACE\t%d:\tA:\t%d\tB:\t%d\tC:\t%d\tAB:\t1\tBC:\t1\tCA:\t1\t*MESH_SMOOTHING\t0\t*MESH_MTLID\t0\r\n",
				t, a, b, c);
		}
		fprintf(f, "\t\t}\r\n");

		/* Texture vertices */
		fprintf(f, "\t\t*MESH_NUMTVERTEX\t%d\r\n", s->numVerts);
		fprintf(f, "\t\t*MESH_TVERTLIST\t{\r\n");
		for (int v = 0; v < s->numVerts; v++) {
			float u = s->st ? s->st[v*2+0] : 0.0f;
			float vt = s->st ? (1.0f - s->st[v*2+1]) : 0.0f;
			fprintf(f, "\t\t\t*MESH_TVERT\t%d\t%f\t%f\t0.0000\r\n", v, u, vt);
		}
		fprintf(f, "\t\t}\r\n");

		/* Texture faces */
		fprintf(f, "\t\t*MESH_NUMTVFACES\t%d\r\n", s->numTris);
		fprintf(f, "\t\t*MESH_TFACELIST\t{\r\n");
		for (int t = 0; t < s->numTris; t++) {
			int a = s->indices[t*3+0];
			int b = s->indices[t*3+1];
			int c = s->indices[t*3+2];
			fprintf(f, "\t\t\t*MESH_TFACE\t%d\t%d\t%d\t%d\r\n", t, a, b, c);
		}
		fprintf(f, "\t\t}\r\n");

		/* Normals */
		if (s->normal) {
			fprintf(f, "\t\t*MESH_NORMALS\t{\r\n");
			for (int t = 0; t < s->numTris; t++) {
				int a = s->indices[t*3+0];
				int b = s->indices[t*3+1];
				int c = s->indices[t*3+2];
				/* face normal = average of vertex normals */
				float fn[3] = {
					(s->normal[a*3+0]+s->normal[b*3+0]+s->normal[c*3+0])/3.0f,
					(s->normal[a*3+1]+s->normal[b*3+1]+s->normal[c*3+1])/3.0f,
					(s->normal[a*3+2]+s->normal[b*3+2]+s->normal[c*3+2])/3.0f
				};
				float len = sqrtf(fn[0]*fn[0]+fn[1]*fn[1]+fn[2]*fn[2]);
				if (len > 1e-6f) { fn[0]/=len; fn[1]/=len; fn[2]/=len; }
				fprintf(f, "\t\t\t*MESH_FACENORMAL\t%d\t%f\t%f\t%f\r\n", t, fn[0], fn[1], fn[2]);
				fprintf(f, "\t\t\t\t*MESH_VERTEXNORMAL\t%d\t%f\t%f\t%f\r\n", a, s->normal[a*3+0], s->normal[a*3+1], s->normal[a*3+2]);
				fprintf(f, "\t\t\t\t*MESH_VERTEXNORMAL\t%d\t%f\t%f\t%f\r\n", b, s->normal[b*3+0], s->normal[b*3+1], s->normal[b*3+2]);
				fprintf(f, "\t\t\t\t*MESH_VERTEXNORMAL\t%d\t%f\t%f\t%f\r\n", c, s->normal[c*3+0], s->normal[c*3+1], s->normal[c*3+2]);
			}
			fprintf(f, "\t\t}\r\n");
		}

		fprintf(f, "\t}\r\n"); /* end MESH */

		fprintf(f, "\t*PROP_MOTIONBLUR\t0\r\n");
		fprintf(f, "\t*PROP_CASTSHADOW\t1\r\n");
		fprintf(f, "\t*PROP_RECVSHADOW\t1\r\n");
		fprintf(f, "\t*MATERIAL_REF\t%d\r\n", si);
		fprintf(f, "}\r\n"); /* end GEOMOBJECT */
	}

	/* Tags as minimal geometry objects */
	for (int ti = 0; ti < m->numTags; ti++) {
		const mc_tag_t *tag = &m->tags[ti];
		fprintf(f, "*GEOMOBJECT\t{\r\n");
		fprintf(f, "\t*NODE_NAME\t\"%s\"\r\n", tag->name);
		fprintf(f, "\t*NODE_TM\t{\r\n");
		fprintf(f, "\t\t*NODE_NAME\t\"%s\"\r\n", tag->name);
		fprintf(f, "\t\t*TM_ROW0\t%f\t%f\t%f\r\n", tag->axis[0][0], tag->axis[0][1], tag->axis[0][2]);
		fprintf(f, "\t\t*TM_ROW1\t%f\t%f\t%f\r\n", tag->axis[1][0], tag->axis[1][1], tag->axis[1][2]);
		fprintf(f, "\t\t*TM_ROW2\t%f\t%f\t%f\r\n", tag->axis[2][0], tag->axis[2][1], tag->axis[2][2]);
		fprintf(f, "\t\t*TM_ROW3\t0\t0\t0\r\n");
		fprintf(f, "\t\t*TM_POS\t%f\t%f\t%f\r\n", tag->origin[0], tag->origin[1], tag->origin[2]);
		fprintf(f, "\t}\r\n");
		/* Emit a degenerate triangle so tag geometry is preserved on round-trip */
		fprintf(f, "\t*MESH\t{\r\n");
		fprintf(f, "\t\t*TIMEVALUE\t0\r\n");
		fprintf(f, "\t\t*MESH_NUMVERTEX\t3\r\n");
		fprintf(f, "\t\t*MESH_NUMFACES\t1\r\n");
		fprintf(f, "\t\t*MESH_VERTEX_LIST\t{\r\n");
		fprintf(f, "\t\t\t*MESH_VERTEX\t0\t%f\t%f\t%f\r\n", tag->origin[0], tag->origin[1], tag->origin[2]);
		fprintf(f, "\t\t\t*MESH_VERTEX\t1\t%f\t%f\t%f\r\n", tag->origin[0]+1.0f, tag->origin[1], tag->origin[2]);
		fprintf(f, "\t\t\t*MESH_VERTEX\t2\t%f\t%f\t%f\r\n", tag->origin[0], tag->origin[1]+1.0f, tag->origin[2]);
		fprintf(f, "\t\t}\r\n");
		fprintf(f, "\t\t*MESH_FACE_LIST\t{\r\n");
		fprintf(f, "\t\t\t*MESH_FACE\t0:\tA:\t0\tB:\t1\tC:\t2\tAB:\t1\tBC:\t1\tCA:\t1\t*MESH_SMOOTHING\t0\t*MESH_MTLID\t0\r\n");
		fprintf(f, "\t\t}\r\n");
		fprintf(f, "\t}\r\n");
		fprintf(f, "}\r\n");
	}

	fclose(f);
	MC_INFO("ase: wrote %s (%d surfaces, %d tags)\n", path, m->numSurfaces, m->numTags);
	return 0;
}
