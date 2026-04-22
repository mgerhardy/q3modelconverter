/*
===========================================================================
modelconverter - Quake3 player-model bundle support.

A Quake3 player lives on disk as three MD3 files plus sidecars:

    head.md3       (typically a single frame, no Q3 tags)
    upper.md3      (BOTH_/TORSO_ animation frames, tag_head, tag_weapon, ...)
    lower.md3      (LEGS_ animation frames, tag_torso)
    animation.cfg  (frame ranges, sex, headoffset)
    *_default.skin / *_<team>.skin / *_<variant>.skin
    *.shader       (optional, picked up by the asset-roots auto-loader)

This module loads the entire bundle into a single mc_model_t and writes
the bundle back out.  Surfaces and tags are stamped with a `part`
("head" / "upper" / "lower") so the splitter can rebuild the right MD3
for each part.  Frame indices stored on each surface follow the
canonical animation.cfg layout: lower-only frames live at indices
[0..nLower), upper-only at [nLower..nLower+nUpper), head frames are
broadcast (single-frame head copied to every output frame).

Concatenating like this means the in-memory model has
numFrames == nLower + nUpper, and on export we slice the per-surface
xyz/normal arrays back into the right MD3.  Animation.cfg row firstFrame
indices are written verbatim; the engine itself subtracts the
LEGS-vs-TORSO offset at runtime.
===========================================================================
*/

#include "mc_common.h"

#include <ctype.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <dirent.h>
#include <sys/stat.h>
#endif

/* ------------------------------------------------------------------ */
/* directory helpers                                                  */
/* ------------------------------------------------------------------ */

static int file_exists(const char *path) {
	FILE *f = fopen(path, "rb");
	if (!f)
		return 0;
	fclose(f);
	return 1;
}

static int dir_exists(const char *path) {
#ifdef _WIN32
	DWORD a = GetFileAttributesA(path);
	return (a != INVALID_FILE_ATTRIBUTES) && (a & FILE_ATTRIBUTE_DIRECTORY);
#else
	struct stat st;
	if (stat(path, &st) != 0)
		return 0;
	return S_ISDIR(st.st_mode);
#endif
}

int mc_is_player_dir(const char *dir) {
	if (!dir || !*dir)
		return 0;
	if (!dir_exists(dir))
		return 0;
	char p[MC_MAX_PATH];
	mc_join_path(p, sizeof(p), dir, "lower.md3");
	if (!file_exists(p))
		return 0;
	mc_join_path(p, sizeof(p), dir, "upper.md3");
	if (!file_exists(p))
		return 0;
	mc_join_path(p, sizeof(p), dir, "head.md3");
	return file_exists(p);
}

/* ------------------------------------------------------------------ */
/* skin variant discovery                                             */
/* ------------------------------------------------------------------ */

static const char *kPlayerParts[3] = {"head", "upper", "lower"};

/* Returns 1 and fills out_part/out_variant when `name` matches the
   "<part>_<variant>.skin" or "<variant>_<part>.skin" pattern that
   appears under wop's player directories (e.g. head_red.skin /
   glow_head_default.skin).  Variant names containing additional
   underscores are preserved verbatim. */
static int classify_skin_filename(const char *name, char *out_part, size_t part_size, char *out_variant,
								   size_t var_size) {
	const char *dot = strrchr(name, '.');
	if (!dot || strcasecmp(dot, ".skin"))
		return 0;
	size_t base_len = (size_t)(dot - name);
	char base[MC_MAX_PATH];
	if (base_len >= sizeof(base))
		base_len = sizeof(base) - 1;
	memcpy(base, name, base_len);
	base[base_len] = 0;

	/* Try "<part>_<variant>" first (head_default, lower_red, ...). */
	for (int i = 0; i < 3; ++i) {
		size_t pl = strlen(kPlayerParts[i]);
		if (base_len > pl + 1 && !strncasecmp(base, kPlayerParts[i], pl) && base[pl] == '_') {
			mc_q_strncpy(out_part, kPlayerParts[i], part_size);
			mc_q_strncpy(out_variant, base + pl + 1, var_size);
			return 1;
		}
	}
	/* Then "<prefix>_<part>_<variant>" / "<variant>_<part>" (glow_head_default,
	   icon_padknight_blue, ...).  We only treat it as a skin if a known
	   part token appears bracketed by underscores. */
	for (int i = 0; i < 3; ++i) {
		const char *needle_start = base;
		size_t pl = strlen(kPlayerParts[i]);
		while (1) {
			const char *m = strstr(needle_start, kPlayerParts[i]);
			if (!m)
				break;
			int leftOk = (m == base) || (m[-1] == '_');
			int rightOk = (m[pl] == 0) || (m[pl] == '_');
			if (leftOk && rightOk) {
				mc_q_strncpy(out_part, kPlayerParts[i], part_size);
				char variant[MC_MAX_PATH];
				size_t off = 0;
				if (m > base) {
					size_t pre = (size_t)(m - base);
					/* strip trailing underscore */
					if (pre > 0 && base[pre - 1] == '_')
						--pre;
					if (pre >= sizeof(variant))
						pre = sizeof(variant) - 1;
					memcpy(variant, base, pre);
					off = pre;
				}
				if (m[pl] == '_') {
					const char *suf = m + pl + 1;
					size_t sl = strlen(suf);
					if (off > 0 && off + 1 < sizeof(variant)) {
						variant[off++] = '_';
					}
					if (off + sl >= sizeof(variant))
						sl = sizeof(variant) - off - 1;
					memcpy(variant + off, suf, sl);
					off += sl;
				}
				variant[off] = 0;
				if (!variant[0])
					mc_q_strncpy(variant, "default", sizeof(variant));
				mc_q_strncpy(out_variant, variant, var_size);
				return 1;
			}
			needle_start = m + 1;
		}
	}
	return 0;
}

#ifdef _WIN32
static void scan_skins(const char *dir, mc_model_t *m) {
	WIN32_FIND_DATAA fd;
	char pat[MC_MAX_PATH];
	snprintf(pat, sizeof(pat), "%s\\*.skin", dir);
	HANDLE h = FindFirstFileA(pat, &fd);
	if (h == INVALID_HANDLE_VALUE)
		return;
	do {
		char part[16], variant[MC_MAX_QPATH];
		if (!classify_skin_filename(fd.cFileName, part, sizeof(part), variant, sizeof(variant)))
			continue;
		char full[MC_MAX_PATH];
		mc_join_path(full, sizeof(full), dir, fd.cFileName);
		mc_skin_variant_t *v = mc_model_add_skin_variant(m, variant, part);
		mc_load_q3skin_variant(full, v);
	} while (FindNextFileA(h, &fd));
	FindClose(h);
}
#else
static void scan_skins(const char *dir, mc_model_t *m) {
	DIR *d = opendir(dir);
	if (!d)
		return;
	struct dirent *de;
	while ((de = readdir(d))) {
		char part[16], variant[MC_MAX_QPATH];
		if (!classify_skin_filename(de->d_name, part, sizeof(part), variant, sizeof(variant)))
			continue;
		char full[MC_MAX_PATH];
		mc_join_path(full, sizeof(full), dir, de->d_name);
		mc_skin_variant_t *v = mc_model_add_skin_variant(m, variant, part);
		mc_load_q3skin_variant(full, v);
	}
	closedir(d);
}
#endif

/* ------------------------------------------------------------------ */
/* Per-part MD3 loader -> stamp surfaces/tags with the part label and  */
/* concat into the bundle model.                                       */
/* ------------------------------------------------------------------ */

/* Append surfaces from `src` into `dst`, preserving xyz/normal/uv/idx.
   Each new surface is allocated with `dst_total_frames` frames; vertex
   data lives at frames [src_frame_start .. src_frame_start+src->numFrames).
   Frames outside that range are filled with the closest valid frame so
   the bundle always has a coherent vertex stream for every output frame. */
static void merge_surfaces(mc_model_t *dst, const mc_model_t *src, const char *part, int lod, int dst_total_frames,
						   int src_frame_start) {
	for (int i = 0; i < src->numSurfaces; ++i) {
		const mc_surface_t *ss = &src->surfaces[i];
		mc_surface_t *ds = mc_model_add_surface(dst);
		if (lod > 0)
			snprintf(ds->name, sizeof(ds->name), "%s_%s_lod%d", part, ss->name, lod);
		else
			snprintf(ds->name, sizeof(ds->name), "%s_%s", part, ss->name);
		mc_q_strncpy(ds->part, part, sizeof(ds->part));
		ds->lod = lod;
		mc_q_strncpy(ds->shader, ss->shader, sizeof(ds->shader));
		mc_q_strncpy(ds->texture, ss->texture, sizeof(ds->texture));
		mc_q_strncpy(ds->normal_map, ss->normal_map, sizeof(ds->normal_map));
		mc_q_strncpy(ds->normal_height_map, ss->normal_height_map, sizeof(ds->normal_height_map));
		mc_q_strncpy(ds->emissive_map, ss->emissive_map, sizeof(ds->emissive_map));
		mc_q_strncpy(ds->mr_map, ss->mr_map, sizeof(ds->mr_map));
		mc_q_strncpy(ds->surfaceparm, ss->surfaceparm, sizeof(ds->surfaceparm));
		ds->two_sided = ss->two_sided;
		ds->alpha_mode = ss->alpha_mode;
		ds->alpha_cutoff = ss->alpha_cutoff;
		memcpy(ds->base_color, ss->base_color, sizeof(ds->base_color));
		memcpy(ds->emissive_factor, ss->emissive_factor, sizeof(ds->emissive_factor));

		mc_surface_alloc(ds, ss->numVerts, ss->numTris, dst_total_frames);

		memcpy(ds->st, ss->st, (size_t)ss->numVerts * 2 * sizeof(float));
		memcpy(ds->indices, ss->indices, (size_t)ss->numTris * 3 * sizeof(int));

		for (int f = 0; f < dst_total_frames; ++f) {
			int sf = f - src_frame_start;
			if (sf < 0)
				sf = 0;
			if (sf >= src->numFrames)
				sf = src->numFrames - 1;
			if (sf < 0)
				sf = 0;
			memcpy(ds->xyz + (size_t)f * ss->numVerts * 3,
				   ss->xyz + (size_t)sf * ss->numVerts * 3, (size_t)ss->numVerts * 3 * sizeof(float));
			memcpy(ds->normal + (size_t)f * ss->numVerts * 3,
				   ss->normal + (size_t)sf * ss->numVerts * 3, (size_t)ss->numVerts * 3 * sizeof(float));
		}
	}
}

static void merge_tags(mc_model_t *dst, const mc_model_t *src, const char *part, int lod, int dst_total_frames,
					   int src_frame_start) {
	if (src->numTags <= 0)
		return;
	for (int t = 0; t < src->numTags; ++t) {
		mc_tag_t *nt = mc_model_add_tag_slot(dst, dst_total_frames);
		const mc_tag_t *seed = &src->tags[t]; /* frame 0 metadata */
		mc_q_strncpy(nt->name, seed->name, sizeof(nt->name));
		mc_q_strncpy(nt->part, part, sizeof(nt->part));
		nt->lod = lod;
		int newTagIdx = dst->numTags - 1;
		for (int f = 0; f < dst_total_frames; ++f) {
			int sf = f - src_frame_start;
			if (sf < 0)
				sf = 0;
			if (sf >= src->numFrames)
				sf = src->numFrames - 1;
			if (sf < 0)
				sf = 0;
			const mc_tag_t *st = &src->tags[(size_t)sf * src->numTags + t];
			mc_tag_t *out = &dst->tags[(size_t)f * dst->numTags + newTagIdx];
			mc_q_strncpy(out->name, st->name, sizeof(out->name));
			mc_q_strncpy(out->part, part, sizeof(out->part));
			out->lod = lod;
			memcpy(out->origin, st->origin, sizeof(out->origin));
			memcpy(out->axis, st->axis, sizeof(out->axis));
		}
	}
}

/* ------------------------------------------------------------------ */
/* Public loader                                                       */
/* ------------------------------------------------------------------ */

int mc_load_player_dir(const char *dir, mc_model_t *out) {
	if (!mc_is_player_dir(dir)) {
		MC_ERR("error: '%s' is not a Q3 player directory (need head/upper/lower.md3)\n", dir);
		return -1;
	}

	mc_model_t head, upper, lower;
	mc_model_init(&head);
	mc_model_init(&upper);
	mc_model_init(&lower);

	char p[MC_MAX_PATH];
	mc_join_path(p, sizeof(p), dir, "lower.md3");
	if (mc_load_md3(p, &lower) != 0) {
		MC_ERR("error: cannot load %s\n", p);
		mc_model_free(&head);
		mc_model_free(&upper);
		mc_model_free(&lower);
		return -1;
	}
	mc_join_path(p, sizeof(p), dir, "upper.md3");
	if (mc_load_md3(p, &upper) != 0) {
		MC_ERR("error: cannot load %s\n", p);
		mc_model_free(&head);
		mc_model_free(&upper);
		mc_model_free(&lower);
		return -1;
	}
	mc_join_path(p, sizeof(p), dir, "head.md3");
	if (mc_load_md3(p, &head) != 0) {
		MC_ERR("error: cannot load %s\n", p);
		mc_model_free(&head);
		mc_model_free(&upper);
		mc_model_free(&lower);
		return -1;
	}

	mc_model_init(out);
	out->is_player_bundle = 1;
	out->fps = 15.0f;
	out->part_numFrames[0] = head.numFrames;
	out->part_numFrames[1] = upper.numFrames;
	out->part_numFrames[2] = lower.numFrames;

	int totalFrames = lower.numFrames + upper.numFrames;
	if (totalFrames < 1)
		totalFrames = 1;

	for (int i = 0; i < totalFrames; ++i) {
		mc_frame_t *fr = mc_model_add_frame(out);
		snprintf(fr->name, sizeof(fr->name), "frame%d", i);
	}

	/* Lower frames occupy [0 .. nLower), upper frames [nLower .. total). */
	merge_surfaces(out, &lower, MC_PART_LOWER, 0, totalFrames, 0);
	merge_tags(out, &lower, MC_PART_LOWER, 0, totalFrames, 0);
	merge_surfaces(out, &upper, MC_PART_UPPER, 0, totalFrames, lower.numFrames);
	merge_tags(out, &upper, MC_PART_UPPER, 0, totalFrames, lower.numFrames);
	/* head is single-frame and broadcast everywhere. */
	merge_surfaces(out, &head, MC_PART_HEAD, 0, totalFrames, 0);
	merge_tags(out, &head, MC_PART_HEAD, 0, totalFrames, 0);

	/* Optional lower-detail LODs: <part>_1.md3, <part>_2.md3 (Q3 limit
	   is MD3_MAX_LODS-1 extra files).  All LODs share the same animation
	   layout as the base part, so we reuse the same frame_start and
	   totalFrames computed above. */
	for (int lod = 1; lod < MD3_MAX_LODS; ++lod) {
		struct { const char *part; int frames; int frame_start; } parts[3] = {
			{ MC_PART_LOWER, lower.numFrames, 0 },
			{ MC_PART_UPPER, upper.numFrames, lower.numFrames },
			{ MC_PART_HEAD,  head.numFrames,  0 },
		};
		for (int pi = 0; pi < 3; ++pi) {
			char lp[MC_MAX_PATH];
			snprintf(lp, sizeof(lp), "%s/%s_%d.md3", dir, parts[pi].part, lod);
			if (!file_exists(lp))
				continue;
			mc_model_t lodm;
			mc_model_init(&lodm);
			if (mc_load_md3(lp, &lodm) != 0) {
				MC_ERR("warning: cannot load LOD %s\n", lp);
				mc_model_free(&lodm);
				continue;
			}
			merge_surfaces(out, &lodm, parts[pi].part, lod, totalFrames, parts[pi].frame_start);
			/* LOD MD3s carry their own tags too, but the engine only
			   uses tags from the base LOD; skip merging them to avoid
			   duplicate-named tag spam in the bundle. */
			mc_model_free(&lodm);
			MC_LOG("player bundle: loaded LOD %d for %s\n", lod, parts[pi].part);
		}
	}

	/* animation.cfg */
	mc_join_path(p, sizeof(p), dir, "animation.cfg");
	if (file_exists(p)) {
		mc_load_animation_cfg(p, out);
	}

	/* skin variants */
	scan_skins(dir, out);

	/* Apply each part's *_default.skin to the merged surfaces so the
	   bundle has working shader paths even before any user-selected
	   variant is chosen.  Without this, surfaces fall back to the MD3
	   surface name (e.g. "upper") and texture lookup fails. */
	for (int i = 0; i < out->numSkins; ++i) {
		const mc_skin_variant_t *sv = &out->skins[i];
		if (strcasecmp(sv->name, "default") != 0)
			continue;
		for (int j = 0; j < sv->numEntries; ++j) {
			const mc_skin_entry_t *en = &sv->entries[j];
			for (int k = 0; k < out->numSurfaces; ++k) {
				mc_surface_t *s = &out->surfaces[k];
				if (strcmp(s->part, sv->part) != 0)
					continue;
				/* Match by the original (un-prefixed) surface name. */
				const char *bare = s->name;
				size_t pl = strlen(sv->part);
				if (!strncmp(bare, sv->part, pl) && bare[pl] == '_')
					bare = bare + pl + 1;
				/* Strip a trailing "_lodN" suffix added by LOD merge so
				   higher-detail LODs inherit the same skin entries. */
				char bareBuf[MC_MAX_QPATH];
				mc_q_strncpy(bareBuf, bare, sizeof(bareBuf));
				char *under = strrchr(bareBuf, '_');
				if (under && !strncmp(under, "_lod", 4)) {
					int isnum = under[4] != 0;
					for (const char *q = under + 4; *q; ++q)
						if (*q < '0' || *q > '9') { isnum = 0; break; }
					if (isnum)
						*under = 0;
				}
				if (!strcasecmp(bareBuf, en->surface)) {
					mc_q_strncpy(s->shader, en->shader, sizeof(s->shader));
					s->texture[0] = 0;
				}
			}
		}
	}

	/* Recompute per-frame bounds across merged surfaces. */
	for (int f = 0; f < out->numFrames; ++f) {
		mc_frame_t *fr = &out->frames[f];
		float mins[3] = {1e30f, 1e30f, 1e30f};
		float maxs[3] = {-1e30f, -1e30f, -1e30f};
		for (int i = 0; i < out->numSurfaces; ++i) {
			const mc_surface_t *s = &out->surfaces[i];
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
		float cx = 0.5f * (mins[0] + maxs[0]);
		float cy = 0.5f * (mins[1] + maxs[1]);
		float cz = 0.5f * (mins[2] + maxs[2]);
		float r2 = 0;
		for (int i = 0; i < out->numSurfaces; ++i) {
			const mc_surface_t *s = &out->surfaces[i];
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
	}

	MC_LOG("player bundle: %d surfaces (%d head / %d upper / %d lower frames)\n", out->numSurfaces,
		   head.numFrames, upper.numFrames, lower.numFrames);

	mc_model_free(&head);
	mc_model_free(&upper);
	mc_model_free(&lower);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Public splitter                                                     */
/* ------------------------------------------------------------------ */

/* Build a single-part view of `m` (head / upper / lower) and write it
   as <dir>/<part>.md3.  The slice of frames written is governed by
   part_numFrames[] when present; if zero we fall back to the full
   in-memory frame range for that part. */
static int write_part_md3(const mc_model_t *m, const char *dir, const char *part, int lod, int frame_start,
						  int frame_count) {
	mc_model_t out;
	mc_model_init(&out);

	if (frame_count < 1)
		frame_count = 1;

	for (int f = 0; f < frame_count; ++f) {
		mc_frame_t *fr = mc_model_add_frame(&out);
		int srcF = frame_start + f;
		if (srcF < 0)
			srcF = 0;
		if (srcF >= m->numFrames)
			srcF = m->numFrames - 1;
		if (srcF >= 0 && m->numFrames > 0) {
			memcpy(fr, &m->frames[srcF], sizeof(*fr));
		}
		snprintf(fr->name, sizeof(fr->name), "frame%d", f);
	}

	int hadAnyForThisLod = 0;

	/* Surfaces. */
	for (int i = 0; i < m->numSurfaces; ++i) {
		const mc_surface_t *ss = &m->surfaces[i];
		if (strcasecmp(ss->part, part))
			continue;
		if (ss->lod != lod)
			continue;
		hadAnyForThisLod = 1;
		mc_surface_t *ds = mc_model_add_surface(&out);
		const char *prefix = part;
		size_t pl = strlen(prefix);
		const char *short_name = ss->name;
		if (!strncasecmp(short_name, prefix, pl) && short_name[pl] == '_')
			short_name += pl + 1;
		/* Strip trailing _lodN suffix added during merge. */
		char nameBuf[MC_MAX_QPATH];
		mc_q_strncpy(nameBuf, short_name, sizeof(nameBuf));
		char *under = strrchr(nameBuf, '_');
		if (under && !strncmp(under, "_lod", 4)) {
			int isnum = under[4] != 0;
			for (const char *q = under + 4; *q; ++q)
				if (*q < '0' || *q > '9') { isnum = 0; break; }
			if (isnum)
				*under = 0;
		}
		mc_q_strncpy(ds->name, nameBuf, sizeof(ds->name));
		mc_q_strncpy(ds->part, part, sizeof(ds->part));
		ds->lod = lod;
		mc_q_strncpy(ds->shader, ss->shader, sizeof(ds->shader));
		mc_q_strncpy(ds->texture, ss->texture, sizeof(ds->texture));
		ds->two_sided = ss->two_sided;
		ds->alpha_mode = ss->alpha_mode;
		ds->alpha_cutoff = ss->alpha_cutoff;
		memcpy(ds->base_color, ss->base_color, sizeof(ds->base_color));
		mc_surface_alloc(ds, ss->numVerts, ss->numTris, frame_count);
		memcpy(ds->st, ss->st, (size_t)ss->numVerts * 2 * sizeof(float));
		memcpy(ds->indices, ss->indices, (size_t)ss->numTris * 3 * sizeof(int));
		for (int f = 0; f < frame_count; ++f) {
			int srcF = frame_start + f;
			if (srcF < 0)
				srcF = 0;
			if (srcF >= m->numFrames)
				srcF = m->numFrames - 1;
			if (srcF < 0)
				srcF = 0;
			memcpy(ds->xyz + (size_t)f * ss->numVerts * 3,
				   ss->xyz + (size_t)srcF * ss->numVerts * 3, (size_t)ss->numVerts * 3 * sizeof(float));
			memcpy(ds->normal + (size_t)f * ss->numVerts * 3,
				   ss->normal + (size_t)srcF * ss->numVerts * 3, (size_t)ss->numVerts * 3 * sizeof(float));
		}
	}

	if (!hadAnyForThisLod) {
		mc_model_free(&out);
		return 1; /* nothing to write for this LOD */
	}

	/* Tags - LOD0 only carries tags (engine ignores tags from LOD>0). */
	if (lod == 0) {
		int partTagSlots = 0;
		int partTagOrigIdx[MD3_MAX_TAGS];
		for (int t = 0; t < m->numTags && partTagSlots < MD3_MAX_TAGS; ++t) {
			const mc_tag_t *seed = &m->tags[t];
			if (strcasecmp(seed->part, part))
				continue;
			if (seed->lod != 0)
				continue;
			mc_tag_t *nt = mc_model_add_tag_slot(&out, frame_count);
			mc_q_strncpy(nt->name, seed->name, sizeof(nt->name));
			mc_q_strncpy(nt->part, part, sizeof(nt->part));
			partTagOrigIdx[partTagSlots] = t;
			++partTagSlots;
		}
		for (int f = 0; f < frame_count; ++f) {
			int srcF = frame_start + f;
			if (srcF < 0)
				srcF = 0;
			if (srcF >= m->numFrames)
				srcF = m->numFrames - 1;
			if (srcF < 0)
				srcF = 0;
			for (int i = 0; i < partTagSlots; ++i) {
				const mc_tag_t *src = &m->tags[(size_t)srcF * m->numTags + partTagOrigIdx[i]];
				mc_tag_t *dst = &out.tags[(size_t)f * out.numTags + i];
				mc_q_strncpy(dst->name, src->name, sizeof(dst->name));
				mc_q_strncpy(dst->part, part, sizeof(dst->part));
				memcpy(dst->origin, src->origin, sizeof(dst->origin));
				memcpy(dst->axis, src->axis, sizeof(dst->axis));
			}
		}
	}

	char path[MC_MAX_PATH];
	if (lod > 0)
		snprintf(path, sizeof(path), "%s/%s_%d.md3", dir, part, lod);
	else
		snprintf(path, sizeof(path), "%s/%s.md3", dir, part);
	int rc = mc_save_md3(path, &out);
	if (rc == 0)
		MC_LOG("wrote %s (%d surf, %d frames, %d tags)\n", path, out.numSurfaces, out.numFrames, out.numTags);
	mc_model_free(&out);
	return rc;
}

int mc_save_player_dir(const char *dir, const mc_model_t *m) {
	mc_ensure_directory(dir);
	if (!dir_exists(dir)) {
		MC_ERR("error: cannot create directory '%s'\n", dir);
		return -1;
	}

	int nHead = m->part_numFrames[0];
	int nUpper = m->part_numFrames[1];
	int nLower = m->part_numFrames[2];

	if (nLower <= 0 && nUpper <= 0) {
		/* No part metadata - assume the whole frame range belongs to lower
		   AND upper (the typical case for a freshly authored bundle). */
		nLower = m->numFrames;
		nUpper = m->numFrames;
	}
	if (nHead <= 0)
		nHead = 1;

	/* lower lives at [0 .. nLower); upper at [nLower .. nLower+nUpper).
	   For each part, walk all LODs present in the model and write
	   <part>.md3 / <part>_1.md3 / <part>_2.md3 as needed. */
	struct { const char *part; int frame_start; int frame_count; } parts[3] = {
		{ MC_PART_LOWER, 0,      nLower },
		{ MC_PART_UPPER, nLower, nUpper },
		{ MC_PART_HEAD,  0,      nHead  },
	};
	for (int pi = 0; pi < 3; ++pi) {
		for (int lod = 0; lod < MD3_MAX_LODS; ++lod) {
			int rc = write_part_md3(m, dir, parts[pi].part, lod, parts[pi].frame_start, parts[pi].frame_count);
			if (rc < 0)
				return -1;
			if (rc > 0 && lod == 0) {
				MC_ERR("error: no surfaces tagged part='%s' lod=0 in model\n", parts[pi].part);
				return -1;
			}
		}
	}

	if (m->numAnimations > 0) {
		char p[MC_MAX_PATH];
		snprintf(p, sizeof(p), "%s/animation.cfg", dir);
		mc_save_animation_cfg(p, m);
		MC_LOG("wrote %s\n", p);
	}

	/* Skin variants */
	for (int i = 0; i < m->numSkins; ++i) {
		const mc_skin_variant_t *v = &m->skins[i];
		if (v->numEntries <= 0)
			continue;
		char p[MC_MAX_PATH];
		const char *part = v->part[0] ? v->part : "model";
		const char *variant = v->name[0] ? v->name : "default";
		snprintf(p, sizeof(p), "%s/%s_%s.skin", dir, part, variant);
		mc_save_q3skin_variant(p, v);
	}

	/* If the model carries shader info, emit a single combined .shader
	   named after the directory's basename so it's easy to load. */
	{
		const char *base = mc_basename(dir);
		char p[MC_MAX_PATH];
		snprintf(p, sizeof(p), "%s/%s.shader", dir, base[0] ? base : "player");
		mc_save_q3shader(p, m);
	}

	return 0;
}
