/*
===========================================================================
modelconverter - 3DS (binary) reader

Reads the binary .3ds format (3D Studio) including materials (texture maps).
Based on quake3/q3data/3dslib.c from id Software / GtkRadiant.

Only loading is supported; saving is not implemented (use glTF or ASE for
round-trip authoring).

Based on the source of q3map2
===========================================================================
*/

#include "mc_common.h"

/* 3DS chunk IDs */
#define C3DS_MAGIC          0x4D4D
#define C3DS_EDIT           0x3D3D
#define C3DS_MESH_VERSION   0x3D3E
#define C3DS_NAMED_OBJECT   0x4000
#define C3DS_TRI_OBJECT     0x4100
#define C3DS_POINT_ARRAY    0x4110
#define C3DS_FACE_ARRAY     0x4120
#define C3DS_MSH_MAT_GROUP  0x4130
#define C3DS_TEX_VERTS      0x4140
#define C3DS_MAT_LIST       0xAFFF
#define C3DS_MAT_NAME       0xA000
#define C3DS_TEXMAP         0xA200
#define C3DS_SPECMAP        0xA204
#define C3DS_OPACMAP        0xA210
#define C3DS_BUMPMAP        0xA230
#define C3DS_MAT_MAPNAME    0xA300
#define C3DS_KEYFRAME       0xB000

/* ---- Binary reader helpers ---- */

typedef struct {
	const unsigned char *data;
	size_t size;
	size_t pos;
} reader_t;

static int r_eof(const reader_t *r) { return r->pos >= r->size; }

static uint16_t r_u16(reader_t *r) {
	if (r->pos + 2 > r->size) { r->pos = r->size; return 0; }
	uint16_t v = (uint16_t)(r->data[r->pos] | (r->data[r->pos+1] << 8));
	r->pos += 2;
	return v;
}

static uint32_t r_u32(reader_t *r) {
	if (r->pos + 4 > r->size) { r->pos = r->size; return 0; }
	uint32_t v = (uint32_t)(r->data[r->pos] | (r->data[r->pos+1]<<8) |
	             (r->data[r->pos+2]<<16) | (r->data[r->pos+3]<<24));
	r->pos += 4;
	return v;
}

static float r_f32(reader_t *r) {
	union { uint32_t u; float f; } cv;
	cv.u = r_u32(r);
	return cv.f;
}

static int16_t r_i16(reader_t *r) { return (int16_t)r_u16(r); }

static void r_skip(reader_t *r, size_t n) {
	if (r->pos + n > r->size) r->pos = r->size;
	else r->pos += n;
}

static size_t r_string(reader_t *r, char *buf, size_t bufsize) {
	size_t i = 0;
	while (r->pos < r->size) {
		char c = (char)r->data[r->pos++];
		if (i < bufsize - 1) buf[i++] = c;
		if (c == 0) break;
	}
	buf[i] = 0;
	return i;
}

/* ---- Intermediate structures ---- */

#define MAX_3DS_MATERIALS 64
#define MAX_3DS_OBJECTS   128

typedef struct {
	char name[128];
	char texture[MC_MAX_PATH];
	char bump[MC_MAX_PATH];
} tds_material_t;

typedef struct {
	char name[128];
	int16_t numFaces;
	int16_t *faces;
} tds_matgroup_t;

typedef struct {
	int numVerts;
	int numFaces;
	int numTexVerts;
	float *verts;       /* numVerts * 3 */
	float *texverts;    /* numTexVerts * 2 */
	int16_t *faces;     /* numFaces * 4 (a,b,c,flags) */
	tds_matgroup_t *matgroups;
	int numMatGroups;
} tds_triobj_t;

typedef struct {
	char name[128];
	tds_triobj_t mesh;
} tds_object_t;

typedef struct {
	tds_material_t materials[MAX_3DS_MATERIALS];
	int numMaterials;
	tds_object_t objects[MAX_3DS_OBJECTS];
	int numObjects;
} tds_scene_t;

static void tds_free_scene(tds_scene_t *s) {
	for (int i = 0; i < s->numObjects; i++) {
		free(s->objects[i].mesh.verts);
		free(s->objects[i].mesh.texverts);
		free(s->objects[i].mesh.faces);
		for (int g = 0; g < s->objects[i].mesh.numMatGroups; g++)
			free(s->objects[i].mesh.matgroups[g].faces);
		free(s->objects[i].mesh.matgroups);
	}
}

/* ---- Chunk-based parser ---- */

static void parse_mapname(reader_t *r, size_t end, char *out, size_t outsize) {
	while (r->pos < end && !r_eof(r)) {
		uint16_t id = r_u16(r);
		uint32_t len = r_u32(r);
		size_t chunk_end = r->pos + (len - 6);
		if (id == C3DS_MAT_MAPNAME) {
			r_string(r, out, outsize);
		} else {
			r_skip(r, len - 6);
		}
		r->pos = chunk_end;
	}
}

static void parse_material(reader_t *r, size_t end, tds_material_t *mat) {
	memset(mat, 0, sizeof(*mat));
	while (r->pos < end && !r_eof(r)) {
		uint16_t id = r_u16(r);
		uint32_t len = r_u32(r);
		size_t chunk_end = r->pos + (len - 6);
		switch (id) {
		case C3DS_MAT_NAME:
			r_string(r, mat->name, sizeof(mat->name));
			break;
		case C3DS_TEXMAP:
			parse_mapname(r, chunk_end, mat->texture, sizeof(mat->texture));
			break;
		case C3DS_BUMPMAP:
			parse_mapname(r, chunk_end, mat->bump, sizeof(mat->bump));
			break;
		default:
			r_skip(r, len - 6);
			break;
		}
		r->pos = chunk_end;
	}
}

static void parse_matgroup(reader_t *r, tds_triobj_t *obj) {
	if (obj->numMatGroups >= 64) return;
	int idx = obj->numMatGroups++;
	obj->matgroups = (tds_matgroup_t *)mc_realloc(obj->matgroups, (size_t)obj->numMatGroups * sizeof(tds_matgroup_t));
	tds_matgroup_t *mg = &obj->matgroups[idx];
	memset(mg, 0, sizeof(*mg));
	r_string(r, mg->name, sizeof(mg->name));
	mg->numFaces = r_i16(r);
	if (mg->numFaces > 0) {
		mg->faces = (int16_t *)mc_malloc((size_t)mg->numFaces * sizeof(int16_t));
		for (int i = 0; i < mg->numFaces; i++)
			mg->faces[i] = r_i16(r);
	}
}

static void parse_triobject(reader_t *r, size_t end, tds_triobj_t *obj) {
	memset(obj, 0, sizeof(*obj));
	while (r->pos < end && !r_eof(r)) {
		uint16_t id = r_u16(r);
		uint32_t len = r_u32(r);
		size_t chunk_end = r->pos + (len - 6);
		switch (id) {
		case C3DS_POINT_ARRAY: {
			obj->numVerts = (int)r_u16(r);
			obj->verts = (float *)mc_malloc((size_t)obj->numVerts * 3 * sizeof(float));
			for (int i = 0; i < obj->numVerts; i++) {
				float x = r_f32(r), y = r_f32(r), z = r_f32(r);
				/* 3DS -> Q3 coordinate swap: x=-y, y=x, z=z */
				obj->verts[i*3+0] = -y;
				obj->verts[i*3+1] = x;
				obj->verts[i*3+2] = z;
			}
			break;
		}
		case C3DS_FACE_ARRAY: {
			obj->numFaces = (int)r_u16(r);
			obj->faces = (int16_t *)mc_malloc((size_t)obj->numFaces * 4 * sizeof(int16_t));
			for (int i = 0; i < obj->numFaces; i++) {
				obj->faces[i*4+0] = r_i16(r); /* a */
				obj->faces[i*4+1] = r_i16(r); /* b */
				obj->faces[i*4+2] = r_i16(r); /* c */
				obj->faces[i*4+3] = r_i16(r); /* flags */
			}
			/* Sub-chunks of FACE_ARRAY (material groups, smooth groups) */
			while (r->pos < chunk_end && !r_eof(r)) {
				uint16_t sid = r_u16(r);
				uint32_t slen = r_u32(r);
				size_t sub_end = r->pos + (slen - 6);
				if (sid == C3DS_MSH_MAT_GROUP)
					parse_matgroup(r, obj);
				else
					r_skip(r, slen - 6);
				r->pos = sub_end;
			}
			break;
		}
		case C3DS_TEX_VERTS: {
			obj->numTexVerts = (int)r_u16(r);
			obj->texverts = (float *)mc_malloc((size_t)obj->numTexVerts * 2 * sizeof(float));
			for (int i = 0; i < obj->numTexVerts; i++) {
				obj->texverts[i*2+0] = r_f32(r);
				obj->texverts[i*2+1] = 1.0f - r_f32(r); /* flip V */
			}
			break;
		}
		default:
			r_skip(r, len - 6);
			break;
		}
		r->pos = chunk_end;
	}
}

static void parse_named_object(reader_t *r, size_t end, tds_scene_t *scene) {
	if (scene->numObjects >= MAX_3DS_OBJECTS) { r->pos = end; return; }
	tds_object_t *obj = &scene->objects[scene->numObjects];
	memset(obj, 0, sizeof(*obj));
	r_string(r, obj->name, sizeof(obj->name));
	while (r->pos < end && !r_eof(r)) {
		uint16_t id = r_u16(r);
		uint32_t len = r_u32(r);
		size_t chunk_end = r->pos + (len - 6);
		if (id == C3DS_TRI_OBJECT)
			parse_triobject(r, chunk_end, &obj->mesh);
		else
			r_skip(r, len - 6);
		r->pos = chunk_end;
	}
	if (obj->mesh.numVerts > 0)
		scene->numObjects++;
}

static void parse_edit_chunk(reader_t *r, size_t end, tds_scene_t *scene) {
	while (r->pos < end && !r_eof(r)) {
		uint16_t id = r_u16(r);
		uint32_t len = r_u32(r);
		size_t chunk_end = r->pos + (len - 6);
		switch (id) {
		case C3DS_MAT_LIST:
			if (scene->numMaterials < MAX_3DS_MATERIALS)
				parse_material(r, chunk_end, &scene->materials[scene->numMaterials++]);
			else
				r_skip(r, len - 6);
			break;
		case C3DS_NAMED_OBJECT:
			parse_named_object(r, chunk_end, scene);
			break;
		default:
			r_skip(r, len - 6);
			break;
		}
		r->pos = chunk_end;
	}
}

static int parse_3ds(const unsigned char *data, size_t size, tds_scene_t *scene) {
	reader_t r = { data, size, 0 };
	uint16_t magic = r_u16(&r);
	uint32_t fileLen = r_u32(&r);
	(void)fileLen;
	if (magic != C3DS_MAGIC) {
		MC_ERR("3ds: invalid magic 0x%04X\n", magic);
		return -1;
	}
	while (!r_eof(&r)) {
		uint16_t id = r_u16(&r);
		uint32_t len = r_u32(&r);
		size_t chunk_end = r.pos + (len - 6);
		if (id == C3DS_EDIT)
			parse_edit_chunk(&r, chunk_end, scene);
		else
			r_skip(&r, len - 6);
		r.pos = chunk_end;
	}
	return 0;
}

/* ---- Find material by name ---- */

static const tds_material_t *find_material(const tds_scene_t *scene, const char *name) {
	for (int i = 0; i < scene->numMaterials; i++) {
		if (!strcmp(scene->materials[i].name, name))
			return &scene->materials[i];
	}
	return NULL;
}

/* ---- Build mc_model_t ---- */

static void compute_normal(const float *v0, const float *v1, const float *v2, float *out) {
	float e1[3] = { v1[0]-v0[0], v1[1]-v0[1], v1[2]-v0[2] };
	float e2[3] = { v2[0]-v0[0], v2[1]-v0[1], v2[2]-v0[2] };
	out[0] = e1[1]*e2[2] - e1[2]*e2[1];
	out[1] = e1[2]*e2[0] - e1[0]*e2[2];
	out[2] = e1[0]*e2[1] - e1[1]*e2[0];
	float len = sqrtf(out[0]*out[0] + out[1]*out[1] + out[2]*out[2]);
	if (len > 1e-6f) { out[0]/=len; out[1]/=len; out[2]/=len; }
	else { out[0]=0; out[1]=0; out[2]=1.0f; }
}

static int build_model(const tds_scene_t *scene, mc_model_t *out) {
	mc_model_init(out);

	mc_frame_t *fr = mc_model_add_frame(out);
	mc_q_strncpy(fr->name, "frame0", sizeof(fr->name));

	for (int oi = 0; oi < scene->numObjects; oi++) {
		const tds_object_t *obj = &scene->objects[oi];
		const tds_triobj_t *mesh = &obj->mesh;

		if (mesh->numFaces <= 0 || mesh->numVerts <= 0)
			continue;

		/* Tags: nodes named tag_* */
		if (strncasecmp(obj->name, "tag_", 4) == 0) {
			mc_tag_t *tag = mc_model_add_tag_slot(out, out->numFrames);
			mc_q_strncpy(tag->name, obj->name, sizeof(tag->name));
			float cx = 0, cy = 0, cz = 0;
			for (int v = 0; v < mesh->numVerts; v++) {
				cx += mesh->verts[v*3+0];
				cy += mesh->verts[v*3+1];
				cz += mesh->verts[v*3+2];
			}
			tag->origin[0] = cx / mesh->numVerts;
			tag->origin[1] = cy / mesh->numVerts;
			tag->origin[2] = cz / mesh->numVerts;
			tag->axis[0][0] = 1; tag->axis[1][1] = 1; tag->axis[2][2] = 1;
			continue;
		}

		/* Determine material from first material group */
		const tds_material_t *mat = NULL;
		if (mesh->numMatGroups > 0)
			mat = find_material(scene, mesh->matgroups[0].name);

		int numTris = mesh->numFaces;
		int numOutVerts = numTris * 3;

		mc_surface_t *surf = mc_model_add_surface(out);
		mc_q_strncpy(surf->name, obj->name, sizeof(surf->name));

		if (mat) {
			if (mat->texture[0]) {
				mc_q_strncpy(surf->texture, mat->texture, sizeof(surf->texture));
				/* shader = texture without extension */
				mc_q_strncpy(surf->shader, mat->texture, sizeof(surf->shader));
				mc_strip_extension(surf->shader);
			} else {
				mc_q_strncpy(surf->shader, mat->name, sizeof(surf->shader));
			}
			if (mat->bump[0])
				mc_q_strncpy(surf->normal_map, mat->bump, sizeof(surf->normal_map));
		}

		mc_surface_alloc(surf, numOutVerts, numTris, 1);

		for (int fi = 0; fi < numTris; fi++) {
			/* 3DS winding: c, b, a (reversed like q3data) */
			int i0 = mesh->faces[fi*4+2];
			int i1 = mesh->faces[fi*4+1];
			int i2 = mesh->faces[fi*4+0];
			if (i0 < 0 || i0 >= mesh->numVerts) i0 = 0;
			if (i1 < 0 || i1 >= mesh->numVerts) i1 = 0;
			if (i2 < 0 || i2 >= mesh->numVerts) i2 = 0;

			int base = fi * 3;
			for (int k = 0; k < 3; k++) {
				surf->xyz[base*3+k]     = mesh->verts[i0*3+k];
				surf->xyz[(base+1)*3+k] = mesh->verts[i1*3+k];
				surf->xyz[(base+2)*3+k] = mesh->verts[i2*3+k];
			}

			/* Texture coords */
			if (mesh->texverts && mesh->numTexVerts > 0) {
				int t0 = i0 < mesh->numTexVerts ? i0 : 0;
				int t1 = i1 < mesh->numTexVerts ? i1 : 0;
				int t2 = i2 < mesh->numTexVerts ? i2 : 0;
				surf->st[base*2+0]     = mesh->texverts[t0*2+0];
				surf->st[base*2+1]     = mesh->texverts[t0*2+1];
				surf->st[(base+1)*2+0] = mesh->texverts[t1*2+0];
				surf->st[(base+1)*2+1] = mesh->texverts[t1*2+1];
				surf->st[(base+2)*2+0] = mesh->texverts[t2*2+0];
				surf->st[(base+2)*2+1] = mesh->texverts[t2*2+1];
			}

			/* Face normal */
			float n[3];
			compute_normal(&surf->xyz[base*3], &surf->xyz[(base+1)*3], &surf->xyz[(base+2)*3], n);
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

/* ---- Public API ---- */

int mc_load_3ds(const char *path, mc_model_t *out) {
	size_t size = 0;
	unsigned char *buf = mc_read_file(path, &size);
	if (!buf) return -1;

	tds_scene_t scene;
	memset(&scene, 0, sizeof(scene));

	int rc = parse_3ds(buf, size, &scene);
	if (rc == 0)
		rc = build_model(&scene, out);

	tds_free_scene(&scene);
	free(buf);

	if (rc == 0)
		MC_INFO("3ds: loaded %s (%d surfaces, %d materials, %d tags)\n",
		        path, out->numSurfaces, scene.numMaterials, out->numTags);
	return rc;
}

/* ---- Binary writer helpers ---- */

typedef struct {
	unsigned char *data;
	size_t size;
	size_t cap;
} wbuf_t;

static void w_grow(wbuf_t *w, size_t need) {
	if (w->size + need <= w->cap) return;
	size_t nc = w->cap ? w->cap * 2 : 4096;
	while (nc < w->size + need) nc *= 2;
	w->data = (unsigned char *)mc_realloc(w->data, nc);
	w->cap = nc;
}

static void w_u16(wbuf_t *w, uint16_t v) {
	w_grow(w, 2);
	w->data[w->size++] = (uint8_t)(v & 0xFF);
	w->data[w->size++] = (uint8_t)(v >> 8);
}

static void w_u32(wbuf_t *w, uint32_t v) {
	w_grow(w, 4);
	w->data[w->size++] = (uint8_t)(v & 0xFF);
	w->data[w->size++] = (uint8_t)((v >> 8) & 0xFF);
	w->data[w->size++] = (uint8_t)((v >> 16) & 0xFF);
	w->data[w->size++] = (uint8_t)(v >> 24);
}

static void w_f32(wbuf_t *w, float f) {
	union { float f; uint32_t u; } cv;
	cv.f = f;
	w_u32(w, cv.u);
}

static void w_str(wbuf_t *w, const char *s) {
	size_t len = strlen(s) + 1;
	w_grow(w, len);
	memcpy(w->data + w->size, s, len);
	w->size += len;
}

static size_t w_chunk_begin(wbuf_t *w, uint16_t id) {
	w_u16(w, id);
	size_t pos = w->size;
	w_u32(w, 0); /* placeholder */
	return pos;
}

static void w_chunk_end(wbuf_t *w, size_t pos) {
	uint32_t len = (uint32_t)(w->size - pos + 2);
	w->data[pos+0] = (uint8_t)(len & 0xFF);
	w->data[pos+1] = (uint8_t)((len >> 8) & 0xFF);
	w->data[pos+2] = (uint8_t)((len >> 16) & 0xFF);
	w->data[pos+3] = (uint8_t)(len >> 24);
}

int mc_save_3ds(const char *path, const mc_model_t *m) {
	wbuf_t w = {NULL, 0, 0};

	size_t magic_pos = w_chunk_begin(&w, C3DS_MAGIC);

	/* Version chunk */
	w_u16(&w, 0x0002);
	w_u32(&w, 10);
	w_u32(&w, 3);

	size_t edit_pos = w_chunk_begin(&w, C3DS_EDIT);

	/* Materials */
	for (int si = 0; si < m->numSurfaces; si++) {
		const mc_surface_t *s = &m->surfaces[si];
		const char *shader = s->shader[0] ? s->shader : s->name;

		size_t mat_pos = w_chunk_begin(&w, C3DS_MAT_LIST);

		size_t mn_pos = w_chunk_begin(&w, C3DS_MAT_NAME);
		w_str(&w, shader);
		w_chunk_end(&w, mn_pos);

		const char *tex = s->texture[0] ? s->texture : (s->shader[0] ? s->shader : NULL);
		if (tex) {
			size_t tm_pos = w_chunk_begin(&w, C3DS_TEXMAP);
			size_t tmn_pos = w_chunk_begin(&w, C3DS_MAT_MAPNAME);
			w_str(&w, tex);
			w_chunk_end(&w, tmn_pos);
			w_chunk_end(&w, tm_pos);
		}

		if (s->normal_map[0]) {
			size_t bm_pos = w_chunk_begin(&w, C3DS_BUMPMAP);
			size_t bmn_pos = w_chunk_begin(&w, C3DS_MAT_MAPNAME);
			w_str(&w, s->normal_map);
			w_chunk_end(&w, bmn_pos);
			w_chunk_end(&w, bm_pos);
		}

		w_chunk_end(&w, mat_pos);
	}

	/* Named objects */
	for (int si = 0; si < m->numSurfaces; si++) {
		const mc_surface_t *s = &m->surfaces[si];
		const char *name = s->name[0] ? s->name : "surface";
		const char *shader = s->shader[0] ? s->shader : s->name;

		size_t no_pos = w_chunk_begin(&w, C3DS_NAMED_OBJECT);
		w_str(&w, name);

		size_t to_pos = w_chunk_begin(&w, C3DS_TRI_OBJECT);

		/* Points: Q3 -> 3DS coord swap (x=y, y=-x, z=z) */
		size_t pa_pos = w_chunk_begin(&w, C3DS_POINT_ARRAY);
		w_u16(&w, (uint16_t)s->numVerts);
		for (int v = 0; v < s->numVerts; v++) {
			w_f32(&w, s->xyz[v*3+1]);
			w_f32(&w, -s->xyz[v*3+0]);
			w_f32(&w, s->xyz[v*3+2]);
		}
		w_chunk_end(&w, pa_pos);

		/* Tex verts */
		if (s->st) {
			size_t tv_pos = w_chunk_begin(&w, C3DS_TEX_VERTS);
			w_u16(&w, (uint16_t)s->numVerts);
			for (int v = 0; v < s->numVerts; v++) {
				w_f32(&w, s->st[v*2+0]);
				w_f32(&w, 1.0f - s->st[v*2+1]);
			}
			w_chunk_end(&w, tv_pos);
		}

		/* Faces (reverse winding: c, b, a) */
		size_t fa_pos = w_chunk_begin(&w, C3DS_FACE_ARRAY);
		w_u16(&w, (uint16_t)s->numTris);
		for (int t = 0; t < s->numTris; t++) {
			w_u16(&w, (uint16_t)s->indices[t*3+2]);
			w_u16(&w, (uint16_t)s->indices[t*3+1]);
			w_u16(&w, (uint16_t)s->indices[t*3+0]);
			w_u16(&w, 0);
		}

		/* Material group */
		size_t mg_pos = w_chunk_begin(&w, C3DS_MSH_MAT_GROUP);
		w_str(&w, shader);
		w_u16(&w, (uint16_t)s->numTris);
		for (int t = 0; t < s->numTris; t++)
			w_u16(&w, (uint16_t)t);
		w_chunk_end(&w, mg_pos);

		w_chunk_end(&w, fa_pos);
		w_chunk_end(&w, to_pos);
		w_chunk_end(&w, no_pos);
	}

	/* Tags as minimal named objects */
	for (int ti = 0; ti < m->numTags; ti++) {
		const mc_tag_t *tag = &m->tags[ti];
		size_t no_pos = w_chunk_begin(&w, C3DS_NAMED_OBJECT);
		w_str(&w, tag->name);

		size_t to_pos = w_chunk_begin(&w, C3DS_TRI_OBJECT);
		size_t pa_pos = w_chunk_begin(&w, C3DS_POINT_ARRAY);
		w_u16(&w, 3);
		w_f32(&w, tag->origin[1]); w_f32(&w, -tag->origin[0]); w_f32(&w, tag->origin[2]);
		w_f32(&w, tag->origin[1]+1.0f); w_f32(&w, -tag->origin[0]); w_f32(&w, tag->origin[2]);
		w_f32(&w, tag->origin[1]); w_f32(&w, -(tag->origin[0]-1.0f)); w_f32(&w, tag->origin[2]);
		w_chunk_end(&w, pa_pos);

		size_t fa_pos = w_chunk_begin(&w, C3DS_FACE_ARRAY);
		w_u16(&w, 1);
		w_u16(&w, 0); w_u16(&w, 1); w_u16(&w, 2); w_u16(&w, 0);
		w_chunk_end(&w, fa_pos);

		w_chunk_end(&w, to_pos);
		w_chunk_end(&w, no_pos);
	}

	w_chunk_end(&w, edit_pos);
	w_chunk_end(&w, magic_pos);

	int rc = mc_write_file(path, w.data, w.size);
	free(w.data);
	if (rc == 0)
		MC_INFO("3ds: wrote %s (%d surfaces, %d tags)\n", path, m->numSurfaces, m->numTags);
	return rc;
}
