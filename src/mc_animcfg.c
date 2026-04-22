/*
===========================================================================
modelconverter - Quake3 player "animation.cfg" reader and writer.

The file is a flat list of frame ranges, in a fixed canonical order
matching bg_public.h.  Each line is:

	<firstFrame> <numFrames> <loopFrames> <fps>		// COMMENT (anim name)

In addition we recognise two header-style directives that some models
ship with:

	sex			m | f | n
	headoffset	X Y Z

The canonical animation names below are emitted by the writer when the
model has the same number of entries as the standard player layout.
For non-standard counts the names recorded on each entry are used.
===========================================================================
*/

#include "mc_common.h"

#include <ctype.h>
#include <string.h>

static const char *kQ3PlayerAnims[] = {
	"BOTH_DEATH1",  "BOTH_DEAD1",   "BOTH_DEATH2",  "BOTH_DEAD2",   "BOTH_DEATH3",  "BOTH_DEAD3",
	"TORSO_GESTURE", "TORSO_ATTACK", "TORSO_ATTACK2",
	"TORSO_DROP",   "TORSO_RAISE",
	"TORSO_STAND",  "TORSO_STAND2",
	"LEGS_WALKCR",  "LEGS_WALK",    "LEGS_RUN",     "LEGS_BACK",    "LEGS_SWIM",
	"LEGS_JUMP",    "LEGS_LAND",
	"LEGS_JUMPB",   "LEGS_LANDB",
	"LEGS_IDLE",    "LEGS_IDLECR",
	"LEGS_TURN",
};
static const int kQ3PlayerAnimCount = (int)(sizeof(kQ3PlayerAnims) / sizeof(kQ3PlayerAnims[0]));

static char *strip_comment(char *line) {
	/* Q3 anim cfg uses // for comments. */
	char *c = strstr(line, "//");
	if (c)
		*c = 0;
	c = strchr(line, '#');
	if (c)
		*c = 0;
	return line;
}

static char *trim(char *s) {
	while (*s && isspace((unsigned char)*s))
		++s;
	char *e = s + strlen(s);
	while (e > s && isspace((unsigned char)e[-1]))
		*--e = 0;
	return s;
}

int mc_load_animation_cfg(const char *path, mc_model_t *m) {
	size_t sz = 0;
	unsigned char *raw = mc_read_file(path, &sz);
	if (!raw)
		return -1;

	/* Reset any prior state so re-loading is idempotent. */
	free(m->animations);
	m->animations = NULL;
	m->numAnimations = 0;
	m->anim_sex = 0;
	m->anim_has_headoffset = 0;
	m->anim_headoffset[0] = m->anim_headoffset[1] = m->anim_headoffset[2] = 0.0f;

	const char *p = (const char *)raw;
	const char *end = p + sz;
	int rowIndex = 0;
	while (p < end) {
		const char *eol = p;
		while (eol < end && *eol != '\n' && *eol != '\r')
			++eol;
		size_t len = (size_t)(eol - p);
		char line[512];
		if (len >= sizeof(line))
			len = sizeof(line) - 1;
		memcpy(line, p, len);
		line[len] = 0;
		p = eol;
		while (p < end && (*p == '\n' || *p == '\r'))
			++p;

		/* Try to lift a comment-derived anim name BEFORE we strip it. */
		char comment_name[64];
		comment_name[0] = 0;
		const char *slash = strstr(line, "//");
		if (slash) {
			const char *cp = slash + 2;
			while (*cp && isspace((unsigned char)*cp))
				++cp;
			int j = 0;
			while (*cp && !isspace((unsigned char)*cp) && *cp != '(' && j + 1 < (int)sizeof(comment_name))
				comment_name[j++] = *cp++;
			comment_name[j] = 0;
		}

		strip_comment(line);
		char *t = trim(line);
		if (!*t)
			continue;

		/* sex / headoffset header lines */
		if (!strncasecmp(t, "sex", 3) && (t[3] == 0 || isspace((unsigned char)t[3]))) {
			char *arg = trim(t + 3);
			if (*arg)
				m->anim_sex = (char)tolower((unsigned char)*arg);
			continue;
		}
		if (!strncasecmp(t, "headoffset", 10) && (t[10] == 0 || isspace((unsigned char)t[10]))) {
			float x = 0, y = 0, z = 0;
			if (sscanf(t + 10, "%f %f %f", &x, &y, &z) >= 1) {
				m->anim_headoffset[0] = x;
				m->anim_headoffset[1] = y;
				m->anim_headoffset[2] = z;
				m->anim_has_headoffset = 1;
			}
			continue;
		}

		/* Frame range row */
		int first = 0, num = 0, loop = 0;
		float fps = 15.0f;
		int n = sscanf(t, "%d %d %d %f", &first, &num, &loop, &fps);
		if (n < 4)
			continue;

		mc_animation_t *a = mc_model_add_animation(m);
		if (!a)
			break;
		a->firstFrame = first;
		a->numFrames = num;
		a->loopFrames = loop;
		a->fps = fps;
		if (comment_name[0]) {
			mc_q_strncpy(a->name, comment_name, sizeof(a->name));
		} else if (rowIndex < kQ3PlayerAnimCount) {
			mc_q_strncpy(a->name, kQ3PlayerAnims[rowIndex], sizeof(a->name));
		} else {
			snprintf(a->name, sizeof(a->name), "ANIM_%d", rowIndex);
		}
		++rowIndex;
	}

	free(raw);
	MC_LOG("parsed %s (%d animation row%s)\n", path, m->numAnimations, m->numAnimations == 1 ? "" : "s");

	/* Range-check what we just parsed against the actual frame count.
	   Player bundles split frames per part, so for those we use each
	   part's frame count rather than the merged total. */
	if (m->numFrames > 0) {
		for (int i = 0; i < m->numAnimations; ++i) {
			const mc_animation_t *a = &m->animations[i];
			int partFrames = m->numFrames;
			if (m->is_player_bundle) {
				int partIdx = -1;
				if (!strncmp(a->name, "LEGS_", 5)) partIdx = 0;
				else if (!strncmp(a->name, "TORSO_", 6) || !strncmp(a->name, "BOTH_", 5)) partIdx = 1;
				if (partIdx >= 0 && m->part_numFrames[partIdx] > 0)
					partFrames = m->part_numFrames[partIdx];
			}
			int last = a->firstFrame + a->numFrames;
			if (a->firstFrame < 0 || a->numFrames < 0 || last > partFrames) {
				MC_LOG("animcfg: anim '%s' frames [%d..%d) out of range (model has %d frame%s) - clamping not applied; emitter will keep the original values\n",
					a->name, a->firstFrame, last, partFrames, partFrames == 1 ? "" : "s");
			}
			if (a->loopFrames < 0 || a->loopFrames > a->numFrames) {
				MC_LOG("animcfg: anim '%s' loopFrames=%d outside [0..%d]; engine will treat as one-shot\n",
					a->name, a->loopFrames, a->numFrames);
			}
			if (a->fps <= 0.0f) {
				MC_LOG("animcfg: anim '%s' fps=%g must be > 0\n", a->name, a->fps);
			}
		}
	}
	return 0;
}

int mc_save_animation_cfg(const char *path, const mc_model_t *m) {
	if (!m || m->numAnimations <= 0)
		return -1;
	FILE *f = fopen(path, "wb");
	if (!f) {
		MC_ERR("error: cannot open '%s' for writing\n", path);
		return -1;
	}
	fprintf(f, "// animation config file\n\n");
	if (m->anim_sex)
		fprintf(f, "sex\t%c\n\n", m->anim_sex);
	fprintf(f, "// first frame, num frames, looping frames, frames per second\n\n");
	if (m->anim_has_headoffset)
		fprintf(f, "headoffset %g %g %g\n\n", m->anim_headoffset[0], m->anim_headoffset[1], m->anim_headoffset[2]);

	for (int i = 0; i < m->numAnimations; ++i) {
		const mc_animation_t *a = &m->animations[i];
		fprintf(f, "%d\t%d\t%d\t%g\t\t// %s\n", a->firstFrame, a->numFrames, a->loopFrames, a->fps, a->name);
	}
	fclose(f);
	return 0;
}
