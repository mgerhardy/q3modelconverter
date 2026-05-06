/*
===========================================================================
modelconverter - command-line entry point.

Converts between glTF/GLB and Quake3-style MD3 / IQM / MDR model formats.
Run `modelconverter --help` for the full option list.
===========================================================================
*/

#include "mc_common.h"

#include <stdio.h>
#include <math.h>
#include <sys/stat.h>

static int path_is_directory(const char *p) {
	if (!p || !*p)
		return 0;
	struct stat st;
	if (stat(p, &st) != 0)
		return 0;
#ifdef _WIN32
	return (st.st_mode & _S_IFDIR) != 0;
#else
	return S_ISDIR(st.st_mode);
#endif
}

static void print_help(const char *prog) {
	printf("modelconverter " MC_VERSION " - WoP/Quake3 model interop tool\n"
		   "\n"
		   "Usage: %s [options] -i <input> -o <output>\n"
		   "       %s --info -i <file>\n"
		   "       %s --validate -i <file>\n"
		   "       %s --player -i <dir> -o <output>\n"
		   "\n"
		   "Supported formats: .gltf .glb .md3 .iqm .mdr .ase\n"
		   "\n"
		   "Options:\n"
		   "  -i, --input  <path>   input model file\n"
		   "  -o, --output <path>   output model file\n"
		   "      --fps <N>         glTF animation sample fps hint (default 15)\n"
		   "      --skin            also emit a .skin file beside the output\n"
		   "      --shader          also emit a .shader file beside the output\n"
		   "      --shader-in <p>   parse a Q3 .shader file and apply it to the loaded model\n"
		   "      --skin-in <p>     parse a Q3 .skin file and override surface->shader mappings\n"
		   "      --no-auto-skin    don't auto-discover <model>_default.skin / <model>.skin\n"
		   "      --no-auto-shader  don't auto-discover .shader files under the asset roots\n"
		   "      --player          force player-bundle mode (auto when input is a directory)\n"
		   "      --subdivide <N>   apply N iterations of Loop (Catmull-Clark for triangle\n"
		   "                        meshes) subdivision before saving; UVs and materials are\n"
		   "                        preserved, all frames are subdivided coherently\n"
		   "      --decimate <R>    QEM (Garland-Heckbert) edge-collapse decimation that\n"
		   "                        keeps R*100%% of triangles per surface (0..1).  Like\n"
		   "                        Blender's Decimate > Collapse; UVs preserved, animation\n"
		   "                        kept coherent across frames\n"
		   "      --gen-lods [N]    auto-generate N additional LOD levels (default 2; max\n"
		   "                        MD3_MAX_LODS-1) using QEM decimation.  Non-player MD3\n"
		   "                        output emits sibling files <name>_1.md3 / <name>_2.md3;\n"
		   "                        player-bundle output writes <part>_<n>.md3 alongside\n"
		   "                        the base part.  glTF outputs encode the LODs in extras.\n"
		   "      --gen-lod-ratios <r1,r2,...>\n"
		   "                        triangle-keep ratios per generated LOD (default 0.5,0.25)\n"
		   "      --asset-root <d>  add a directory to search for textures referenced by the\n"
		   "                        model (repeatable; defaults are derived from the input path,\n"
		   "                        looking for the nearest *.pk3dir / wop / xmas ancestor)\n"
		   "      --shader-path <d> add a directory to scan recursively for *.shader files\n"
		   "                        beyond the asset roots (repeatable)\n"
		   "      --shader-depth <n> max recursion depth for shader auto-discovery (default 6)\n"
		   "      --info            print info about <input> and exit\n"
		   "      --validate        run format limit + sanity checks on <input> and exit non-zero\n"
		   "                        if any issue is found (no output written)\n"
		   "  -v, --verbose         verbose progress output\n"
		   "  -q, --quiet           suppress info/debug output (errors only)\n"
		   "  -h, --help            this help\n",
		   prog, prog, prog, prog);
}

static void print_info(const mc_model_t *m, const char *path) {
	printf("file: %s\n", path);
	if (m->is_player_bundle) {
		printf("  player   : yes (head=%d, upper=%d, lower=%d frames)\n", m->part_numFrames[0],
			   m->part_numFrames[1], m->part_numFrames[2]);
	}
	printf("  surfaces : %d\n", m->numSurfaces);
	printf("  frames   : %d\n", m->numFrames);
	printf("  tags     : %d\n", m->numTags);
	if (m->numAnimations > 0)
		printf("  anims    : %d\n", m->numAnimations);
	if (m->numSkins > 0)
		printf("  skins    : %d\n", m->numSkins);
	if (m->numJoints > 0)
		printf("  joints   : %d\n", m->numJoints);
	if (m->numLODs > 1)
		printf("  lods     : %d\n", m->numLODs);
	for (int i = 0; i < m->numSurfaces; ++i) {
		const mc_surface_t *s = &m->surfaces[i];
		printf("  [surf %d] name='%s' part='%s' lod=%d shader='%s' verts=%d tris=%d%s\n", i, s->name, s->part, s->lod,
			   s->shader, s->numVerts, s->numTris,
			   (s->blendIndices && s->blendWeights) ? " skinned" : "");
	}
	for (int i = 0; i < m->numTags; ++i) {
		const mc_tag_t *t = &m->tags[i];
		printf("  [tag  %d] name='%s' part='%s' lod=%d origin=(%.3f %.3f %.3f)\n", i, t->name, t->part, t->lod,
			   t->origin[0], t->origin[1], t->origin[2]);
		/* Show drift between frame 0 and the last frame so per-frame
		   tag animation regressions surface in --info diffs. */
		if (m->numFrames > 1 && m->numTags > 0) {
			const mc_tag_t *tl = &m->tags[(size_t)(m->numFrames - 1) * m->numTags + i];
			float dx = tl->origin[0] - t->origin[0];
			float dy = tl->origin[1] - t->origin[1];
			float dz = tl->origin[2] - t->origin[2];
			float drift = (float)sqrt((double)(dx*dx + dy*dy + dz*dz));
			printf("           lastframe=(%.3f %.3f %.3f) drift=%.4f\n",
				   tl->origin[0], tl->origin[1], tl->origin[2], drift);
		}
	}
	for (int i = 0; i < m->numAnimations; ++i) {
		const mc_animation_t *a = &m->animations[i];
		printf("  [anim %d] %-16s first=%d num=%d loop=%d fps=%g\n", i, a->name, a->firstFrame, a->numFrames,
			   a->loopFrames, a->fps);
	}
	for (int i = 0; i < m->numSkins; ++i) {
		const mc_skin_variant_t *v = &m->skins[i];
		printf("  [skin %d] %s_%s (%d entries)\n", i, v->part, v->name, v->numEntries);
	}
}

/* Run cross-format limit checks against the loaded model.  Reports every
   issue found (does not stop on first hit) and returns non-zero if any
   problem was detected.  Intended for use under --validate so CI can
   gate on a clean run. */
static int validate_model(const mc_model_t *m, const char *path) {
	int issues = 0;
	if (m->numSurfaces > MD3_MAX_SURFACES)
		printf("validate: %s: MD3 surfaces %d > limit %d\n", path, m->numSurfaces, MD3_MAX_SURFACES), ++issues;
	if (m->numTags > MD3_MAX_TAGS)
		printf("validate: %s: MD3 tags %d > limit %d\n", path, m->numTags, MD3_MAX_TAGS), ++issues;
	if (m->numFrames > MD3_MAX_FRAMES)
		printf("validate: %s: MD3 frames %d > limit %d (use MDR/IQM for >1024 frames)\n",
			path, m->numFrames, MD3_MAX_FRAMES), ++issues;
	for (int i = 0; i < m->numSurfaces; ++i) {
		const mc_surface_t *s = &m->surfaces[i];
		if (s->numVerts > MD3_MAX_VERTS)
			printf("validate: %s: surface '%s' verts %d > MD3 limit %d\n",
				path, s->name, s->numVerts, MD3_MAX_VERTS), ++issues;
		if (s->numTris > MD3_MAX_TRIANGLES)
			printf("validate: %s: surface '%s' tris %d > MD3 limit %d\n",
				path, s->name, s->numTris, MD3_MAX_TRIANGLES), ++issues;
		if (s->numVerts <= 0)
			printf("validate: %s: surface '%s' has 0 verts\n", path, s->name), ++issues;
		if (s->numTris <= 0)
			printf("validate: %s: surface '%s' has 0 triangles\n", path, s->name), ++issues;
		if (s->lod < 0 || s->lod >= MD3_MAX_LODS)
			printf("validate: %s: surface '%s' lod=%d outside [0..%d)\n",
				path, s->name, s->lod, MD3_MAX_LODS), ++issues;
		/* Triangle index range. */
		if (s->indices) {
			for (int t = 0; t < s->numTris; ++t) {
				for (int k = 0; k < 3; ++k) {
					int v = s->indices[t * 3 + k];
					if (v < 0 || v >= s->numVerts) {
						printf("validate: %s: surface '%s' tri %d idx %d out of range [0..%d)\n",
							path, s->name, t, v, s->numVerts);
						++issues;
						goto next_surf;
					}
				}
			}
		}
		next_surf: ;
	}
	/* animation.cfg: only sanity-check fps.  Frame ranges in
	   animation.cfg are absolute into the *merged* player frame stream
	   with a per-part offset (legsSkip), so a flat firstFrame+numFrames
	   bounds check produces false positives on real assets. */
	for (int i = 0; i < m->numAnimations; ++i) {
		const mc_animation_t *a = &m->animations[i];
		if (a->fps <= 0.0f)
			printf("validate: %s: anim '%s' fps=%g must be > 0\n", path, a->name, a->fps), ++issues;
	}
	/* Joint pose array consistency. */
	if (m->numJoints > 0 && m->numFrames > 0 && !m->jointPoses)
		printf("validate: %s: numJoints=%d but jointPoses is NULL\n", path, m->numJoints), ++issues;
	for (int j = 0; j < m->numJoints; ++j) {
		const mc_joint_t *jt = &m->joints[j];
		if (jt->parent >= j)
			printf("validate: %s: joint '%s' parent=%d must reference an earlier joint\n",
				path, jt->name, jt->parent), ++issues;
	}
	if (issues == 0)
		printf("validate: %s: OK\n", path);
	else
		printf("validate: %s: %d issue%s found\n", path, issues, issues == 1 ? "" : "s");
	return issues == 0 ? 0 : 1;
}

int main(int argc, char **argv) {
	/* Subcommand sugar: rewrite leading positional verbs into the
	   canonical -i / -o / --validate / --info / --player flag form so
	   the rest of the parser stays a single code path.

	     modelconverter info <file>
	     modelconverter validate <file>
	     modelconverter convert <in> <out>
	     modelconverter player <dir> <out.glb>

	   Everything else (-i / -o / --info / --player) keeps working
	   unchanged for backwards compatibility. */
	static char *rewritten[64];
	if (argc >= 2 && argv[1][0] != '-') {
		const char *verb = argv[1];
		/* Per-subcommand --help. */
		int want_subcmd_help = (argc >= 3 && (!strcmp(argv[2], "--help") || !strcmp(argv[2], "-h")));
		if (!want_subcmd_help && argc == 2)
			want_subcmd_help = 1; /* bare "modelconverter info" with no file */
		if (want_subcmd_help) {
			if (!strcmp(verb, "info")) {
				printf("Usage: %s info <file> [-v]\n\nPrint summary information about a model file.\n", argv[0]);
				return 0;
			} else if (!strcmp(verb, "validate")) {
				printf("Usage: %s validate <file> [-v]\n\nRun format-limit and sanity checks. Exits non-zero on issues.\n", argv[0]);
				return 0;
			} else if (!strcmp(verb, "convert")) {
				printf("Usage: %s convert <input> <output> [options]\n\n"
					   "Options:\n"
					   "  --fps <N>          glTF animation sample rate (default 15)\n"
					   "  --skin             emit a .skin sidecar\n"
					   "  --shader           emit a .shader sidecar\n"
					   "  --shader-in <p>    apply an existing .shader file\n"
					   "  --skin-in <p>      apply an existing .skin file\n"
					   "  --subdivide <N>    Loop subdivision iterations\n"
					   "  --decimate <R>     keep R*100%% of triangles\n"
					   "  --gen-lods [N]     auto-generate N LOD levels\n"
					   "  --asset-root <d>   texture search directory\n"
					   "  -v, --verbose      verbose output\n"
					   "  -q, --quiet        errors only\n", argv[0]);
				return 0;
			} else if (!strcmp(verb, "player")) {
				printf("Usage: %s player <dir|file> <output> [options]\n\n"
					   "Convert a Q3 player bundle (head/upper/lower.md3 + animation.cfg).\n"
					   "Input can be a directory or a single .glb containing all parts.\n\n"
					   "Options: same as 'convert' plus --no-auto-skin, --no-auto-shader.\n", argv[0]);
				return 0;
			}
			/* Unknown verb with --help: fall through to global help. */
		}
		int n = 0;
		rewritten[n++] = argv[0];
		if (!strcmp(verb, "info") && argc >= 3) {
			rewritten[n++] = (char *)"--info";
			rewritten[n++] = (char *)"-i";
			rewritten[n++] = argv[2];
			for (int i = 3; i < argc && n < 63; ++i) rewritten[n++] = argv[i];
		} else if (!strcmp(verb, "validate") && argc >= 3) {
			rewritten[n++] = (char *)"--validate";
			rewritten[n++] = (char *)"-i";
			rewritten[n++] = argv[2];
			for (int i = 3; i < argc && n < 63; ++i) rewritten[n++] = argv[i];
		} else if (!strcmp(verb, "convert") && argc >= 4) {
			rewritten[n++] = (char *)"-i";
			rewritten[n++] = argv[2];
			rewritten[n++] = (char *)"-o";
			rewritten[n++] = argv[3];
			for (int i = 4; i < argc && n < 63; ++i) rewritten[n++] = argv[i];
		} else if (!strcmp(verb, "player") && argc >= 4) {
			rewritten[n++] = (char *)"--player";
			rewritten[n++] = (char *)"-i";
			rewritten[n++] = argv[2];
			rewritten[n++] = (char *)"-o";
			rewritten[n++] = argv[3];
			for (int i = 4; i < argc && n < 63; ++i) rewritten[n++] = argv[i];
		} else {
			n = 0; /* unknown verb - leave argv alone */
		}
		if (n > 0) {
			argc = n;
			argv = rewritten;
		}
	}
	const char *input = NULL;
	const char *output = NULL;
	const char *shader_in = NULL;
	const char *skin_in = NULL;
	float fps = 15.0f;
	int want_skin = 0;
	int want_shader = 0;
	int info_only = 0;
	int validate_only = 0;
	int no_auto_skin = 0;
	int no_auto_shader = 0;
	int player_mode = 0;
	int subdivide_levels = 0;
	float decimate_ratio = 1.0f;
	int gen_lods = 0;            /* number of additional LOD levels to generate (0 = off) */
	float gen_lod_ratios[8] = {0.5f, 0.25f, 0.125f, 0.0625f, 0.03125f, 0.015625f, 0.0078125f, 0.00390625f};

#define MC_MAX_ROOTS 32
	const char *asset_roots[MC_MAX_ROOTS + 1] = {0};
	int num_roots = 0;

	for (int i = 1; i < argc; ++i) {
		const char *a = argv[i];
		if (!strcmp(a, "-h") || !strcmp(a, "--help")) {
			print_help(argv[0]);
			return 0;
		} else if (!strcmp(a, "-v") || !strcmp(a, "--verbose")) {
			mc_verbose = 1;
		} else if (!strcmp(a, "-q") || !strcmp(a, "--quiet")) {
			mc_verbose = -1;
		} else if ((!strcmp(a, "-i") || !strcmp(a, "--input")) && i + 1 < argc) {
			input = argv[++i];
		} else if ((!strcmp(a, "-o") || !strcmp(a, "--output")) && i + 1 < argc) {
			output = argv[++i];
		} else if (!strcmp(a, "--fps") && i + 1 < argc) {
			fps = (float)atof(argv[++i]);
		} else if (!strcmp(a, "--skin")) {
			want_skin = 1;
		} else if (!strcmp(a, "--shader")) {
			want_shader = 1;
		} else if (!strcmp(a, "--shader-in") && i + 1 < argc) {
			shader_in = argv[++i];
		} else if (!strcmp(a, "--skin-in") && i + 1 < argc) {
			skin_in = argv[++i];
		} else if (!strcmp(a, "--no-auto-skin")) {
			no_auto_skin = 1;
		} else if (!strcmp(a, "--no-auto-shader")) {
			no_auto_shader = 1;
		} else if (!strcmp(a, "--player")) {
			player_mode = 1;
		} else if (!strcmp(a, "--subdivide") && i + 1 < argc) {
			subdivide_levels = atoi(argv[++i]);
			if (subdivide_levels < 0) subdivide_levels = 0;
		} else if (!strcmp(a, "--decimate") && i + 1 < argc) {
			decimate_ratio = (float)atof(argv[++i]);
			if (decimate_ratio < 0.0f) decimate_ratio = 0.0f;
			if (decimate_ratio > 1.0f) decimate_ratio = 1.0f;
		} else if (!strcmp(a, "--gen-lods")) {
			/* Optional integer argument: number of LODs to generate (default 2). */
			gen_lods = 2;
			if (i + 1 < argc && argv[i + 1][0] >= '0' && argv[i + 1][0] <= '9') {
				gen_lods = atoi(argv[++i]);
			}
			if (gen_lods < 1) gen_lods = 1;
			if (gen_lods > MD3_MAX_LODS - 1) gen_lods = MD3_MAX_LODS - 1;
		} else if (!strcmp(a, "--gen-lod-ratios") && i + 1 < argc) {
			/* Comma-separated ratios, one per LOD level. */
			const char *s = argv[++i];
			int idx = 0;
			while (*s && idx < 8) {
				gen_lod_ratios[idx++] = (float)atof(s);
				while (*s && *s != ',') ++s;
				if (*s == ',') ++s;
			}
		} else if (!strcmp(a, "--asset-root") && i + 1 < argc) {
			if (num_roots < MC_MAX_ROOTS)
				asset_roots[num_roots++] = argv[++i];
			else
				++i;
		} else if (!strcmp(a, "--shader-path") && i + 1 < argc) {
			mc_q3shader_add_search_path(argv[++i]);
		} else if (!strcmp(a, "--shader-depth") && i + 1 < argc) {
			mc_q3shader_set_max_depth(atoi(argv[++i]));
		} else if (!strcmp(a, "--info")) {
			info_only = 1;
		} else if (!strcmp(a, "--validate")) {
			info_only = 1; /* implies no output write */
			validate_only = 1;
		} else {
			MC_ERR("error: unknown argument '%s'\n", a);
			print_help(argv[0]);
			return 2;
		}
	}

	if (!input) {
		MC_ERR("error: --input is required\n");
		print_help(argv[0]);
		return 2;
	}
	if (!info_only && !output) {
		MC_ERR("error: --output is required (or use --info)\n");
		return 2;
	}

	/* Auto-detect player-bundle mode when the input is a directory. */
	if (path_is_directory(input)) {
		if (mc_is_player_dir(input)) {
			player_mode = 1;
		} else {
			MC_ERR("error: input directory '%s' is not a Q3 player layout (need head/upper/lower.md3)\n", input);
			return 2;
		}
	}

	mc_format_t inFmt = MC_FMT_UNKNOWN;
	if (!player_mode || !path_is_directory(input)) {
		inFmt = mc_guess_format(input);
		if (inFmt == MC_FMT_UNKNOWN && !player_mode) {
			MC_ERR("error: cannot detect input format from extension: %s\n", input);
			return 2;
		}
	}

	mc_model_t model;
	mc_model_init(&model);

	int rc = -1;
	if (player_mode && path_is_directory(input)) {
		rc = mc_load_player_dir(input, &model);
	} else {
		switch (inFmt) {
		case MC_FMT_GLTF:
		case MC_FMT_GLB:
			rc = mc_load_gltf(input, &model, fps);
			break;
		case MC_FMT_MD3:
			rc = mc_load_md3(input, &model);
			/* Auto-load animation.cfg next to the MD3 if present. */
			{
				char dir[MC_MAX_PATH];
				mc_dirname(input, dir, sizeof(dir));
				char ap[MC_MAX_PATH];
				snprintf(ap, sizeof(ap), "%s/animation.cfg", dir);
				FILE *af = fopen(ap, "rb");
				if (af) {
					fclose(af);
					mc_load_animation_cfg(ap, &model);
				}
			}
			break;
		case MC_FMT_IQM:
			rc = mc_load_iqm(input, &model);
			break;
		case MC_FMT_MDR:
			rc = mc_load_mdr(input, &model);
			break;
		case MC_FMT_ASE:
			rc = mc_load_ase(input, &model);
			break;
		default:
			break;
		}
	}
	if (rc != 0) {
		MC_ERR("error: failed to load %s\n", input);
		mc_model_free(&model);
		return 1;
	}

	if (shader_in) {
		if (mc_load_q3shader(shader_in, &model) != 0) {
			MC_ERR("warning: could not parse shader file %s\n", shader_in);
		}
	}

	/* Build asset roots and run auto-discovery for .skin and .shader BEFORE
	   subdivide/decimate so per-surface flags (like `deformVertexes
	   autosprite`) are known to the topology passes. */
	if (num_roots == 0 && input) {
		static char autoRoots[8][512];
		char dir[512];
		mc_dirname(input, dir, sizeof(dir));
		int slot = 0;
		for (int up = 0; up < 8 && dir[0] && slot < MC_MAX_ROOTS; ++up) {
			mc_q_strncpy(autoRoots[slot], dir, sizeof(autoRoots[slot]));
			asset_roots[num_roots++] = autoRoots[slot];
			slot++;
			char parent[512];
			mc_dirname(dir, parent, sizeof(parent));
			if (!parent[0] || !strcmp(parent, dir))
				break;
			mc_q_strncpy(dir, parent, sizeof(dir));
		}
		asset_roots[num_roots] = NULL;
	}

	if (skin_in) {
		if (mc_load_q3skin(skin_in, &model) != 0)
			MC_ERR("warning: could not parse skin file %s\n", skin_in);
	} else if (!no_auto_skin && !player_mode) {
		char base[MC_MAX_PATH];
		mc_q_strncpy(base, input, sizeof(base));
		mc_strip_extension(base);
		const char *suffixes[] = {"_default.skin", ".skin", NULL};
		for (int s = 0; suffixes[s]; ++s) {
			char cand[MC_MAX_PATH];
			snprintf(cand, sizeof(cand), "%s%s", base, suffixes[s]);
			FILE *f = fopen(cand, "rb");
			if (f) {
				fclose(f);
				mc_load_q3skin(cand, &model);
				break;
			}
		}
	}

	if (!shader_in && !no_auto_shader) {
		mc_autoload_shaders(asset_roots, &model);
	}

	if (info_only) {
		print_info(&model, input);
		rc = 0;
		if (validate_only) {
			rc = validate_model(&model, input);
		}
		mc_model_free(&model);
		return rc;
	}

	if (subdivide_levels > 0) {
		MC_LOG("subdiv: applying %d Loop subdivision iteration(s)\n", subdivide_levels);
		if (mc_subdivide(&model, subdivide_levels) != 0) {
			MC_ERR("error: subdivision failed\n");
			mc_model_free(&model);
			return 1;
		}
	}

	if (decimate_ratio < 1.0f) {
		MC_LOG("decimate: keeping %.1f%% of triangles per surface\n", decimate_ratio * 100.0f);
		if (mc_decimate(&model, decimate_ratio) != 0) {
			MC_ERR("error: decimation failed\n");
			mc_model_free(&model);
			return 1;
		}
	}

	if (gen_lods > 0) {
		MC_LOG("gen-lods: generating %d additional LOD level(s)\n", gen_lods);
		if (mc_gen_lods(&model, gen_lods, gen_lod_ratios) != 0) {
			MC_ERR("error: LOD generation failed\n");
			mc_model_free(&model);
			return 1;
		}
	}

	mc_format_t outFmt = MC_FMT_UNKNOWN;
	int output_is_player_dir = 0;
	/* Promote to player mode automatically when the loader detected (or
	   inferred from naming conventions) a head/upper/lower bundle. */
	if (model.is_player_bundle && !player_mode) {
		player_mode = 1;
	}
	if (player_mode && (path_is_directory(output) ||
						(output[strlen(output) - 1] == '/' || output[strlen(output) - 1] == '\\') ||
						!strrchr(output, '.'))) {
		output_is_player_dir = 1;
	} else {
		outFmt = mc_guess_format(output);
		if (outFmt == MC_FMT_UNKNOWN) {
			MC_ERR("error: cannot detect output format from extension: %s\n", output);
			mc_model_free(&model);
			return 2;
		}
	}

	/* If the user didn't supply --asset-root, derive sensible defaults so
	   referenced textures can actually be found and embedded.  The list
	   we add is, in order:
		 1. The input file's directory (relative texture paths often live
			next to the model).
		 2. Each ancestor directory walking up from the input, stopping
			when we exhaust 8 levels.  This catches the typical case of a
			model living under wop/models/<mapobj>/foo.md3 with textures
			under wop/textures/<mapobj>/.
	*/
	MC_LOG("converting %s -> %s (%s -> %s, %d surfaces, %d frames, %d tags)\n", input, output,
		   player_mode && path_is_directory(input) ? "playerdir" : mc_format_name(inFmt),
		   output_is_player_dir ? "playerdir" : mc_format_name(outFmt), model.numSurfaces, model.numFrames,
		   model.numTags);

	int wrc = -1;
	if (output_is_player_dir) {
		wrc = mc_save_player_dir(output, &model);
	} else {
		switch (outFmt) {
		case MC_FMT_MD3:
			{
				int maxLod = 0;
				for (int i = 0; i < model.numSurfaces; ++i) {
					if (model.surfaces[i].lod > maxLod) maxLod = model.surfaces[i].lod;
				}
				wrc = 0;
				for (int L = 0; L <= maxLod && wrc == 0; ++L) {
					mc_model_t lodm;
					mc_model_init(&lodm);
					lodm.fps = model.fps;
					for (int f = 0; f < model.numFrames; ++f) {
						mc_frame_t *fr = mc_model_add_frame(&lodm);
						*fr = model.frames[f];
					}
					if (L == 0 && model.numTags > 0) {
						lodm.numTags = model.numTags;
						lodm.tags = model.tags;
					}
					int nMatch = 0;
					for (int i = 0; i < model.numSurfaces; ++i) {
						if (model.surfaces[i].lod != L) continue;
						const mc_surface_t *src = &model.surfaces[i];
						mc_surface_t *dst = mc_model_add_surface(&lodm);
						*dst = *src;
						dst->lod = 0;
						nMatch++;
					}
					if (L == 0 || nMatch > 0) {
						char lodPath[MC_MAX_PATH];
						if (L == 0) {
							mc_q_strncpy(lodPath, output, sizeof(lodPath));
						} else {
							char base[MC_MAX_PATH];
							mc_q_strncpy(base, output, sizeof(base));
							mc_strip_extension(base);
							snprintf(lodPath, sizeof(lodPath), "%s_%d.md3", base, L);
						}
						wrc = mc_save_md3(lodPath, &lodm);
						if (wrc == 0 && L > 0) {
							MC_LOG("wrote %s (LOD %d, %d surfaces)\n", lodPath, L, nMatch);
						}
					}
					/* Release only arrays we own (surfaces/frames are shallow copies). */
					if (L == 0) lodm.tags = NULL;
					free(lodm.frames);
					free(lodm.surfaces);
				}
				if (wrc == 0 && model.numAnimations > 0) {
					char ap[MC_MAX_PATH];
					char dir[MC_MAX_PATH];
					mc_dirname(output, dir, sizeof(dir));
					snprintf(ap, sizeof(ap), "%s/animation.cfg", dir);
					mc_save_animation_cfg(ap, &model);
					MC_LOG("wrote %s\n", ap);
				}
			}
			break;
		case MC_FMT_GLTF:
			wrc = mc_save_gltf(output, &model, 0, asset_roots);
			break;
		case MC_FMT_GLB:
			wrc = mc_save_gltf(output, &model, 1, asset_roots);
			break;
		case MC_FMT_IQM:
			wrc = mc_save_iqm(output, &model);
			break;
		case MC_FMT_MDR:
			wrc = mc_save_mdr(output, &model);
			break;
		case MC_FMT_ASE:
			wrc = mc_save_ase(output, &model);
			break;
		default:
			break;
		}
	}
	if (wrc != 0) {
		MC_ERR("error: failed to write %s\n", output);
		mc_model_free(&model);
		return 1;
	}

	if (want_skin) {
		char skinPath[MC_MAX_PATH];
		mc_q_strncpy(skinPath, output, sizeof(skinPath));
		mc_strip_extension(skinPath);
		size_t l = strlen(skinPath);
		if (l + 6 < sizeof(skinPath))
			memcpy(skinPath + l, ".skin", 6);
		if (mc_save_skin(skinPath, &model) == 0) {
			MC_LOG("wrote %s\n", skinPath);
		}
	}

	if (want_shader) {
		char shaderPath[MC_MAX_PATH];
		mc_q_strncpy(shaderPath, output, sizeof(shaderPath));
		mc_strip_extension(shaderPath);
		size_t l = strlen(shaderPath);
		if (l + 8 < sizeof(shaderPath))
			memcpy(shaderPath + l, ".shader", 8);
		mc_save_q3shader(shaderPath, &model);
	}

	if (mc_verbose)
		print_info(&model, output);

	mc_model_free(&model);
	return 0;
}
