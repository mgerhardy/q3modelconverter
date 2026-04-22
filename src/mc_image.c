/*
===========================================================================
modelconverter - image helpers (TGA/etc -> PNG conversion for glTF).

The glTF 2.0 spec only mandates PNG and JPEG image support.  Q3 assets
typically use TGA, which Blender and most online viewers refuse to
sample.  This module decodes any stb_image-supported source (TGA, BMP,
GIF, PSD, PIC, PNM, JPEG, PNG) and re-encodes it as PNG so the embedded
texture is readable everywhere.
===========================================================================
*/

#include "mc_common.h"

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-prototypes"
#pragma GCC diagnostic ignored "-Wstrict-prototypes"
#endif

#define STB_IMAGE_IMPLEMENTATION
#define STBI_NO_HDR
#define STBI_NO_LINEAR
#define STBI_NO_THREAD_LOCALS
#include "stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

typedef struct {
	unsigned char *data;
	size_t size;
	size_t cap;
} mc_blob_t;

static void blob_write(void *ctx, void *data, int size) {
	mc_blob_t *b = (mc_blob_t *)ctx;
	if (b->size + (size_t)size > b->cap) {
		size_t nc = b->cap ? b->cap * 2 : 4096;
		while (nc < b->size + (size_t)size)
			nc *= 2;
		b->data = (unsigned char *)realloc(b->data, nc);
		b->cap = nc;
	}
	memcpy(b->data + b->size, data, (size_t)size);
	b->size += (size_t)size;
}

unsigned char *mc_image_to_png(const unsigned char *src, size_t src_size, size_t *out_size) {
	int w = 0, h = 0, comp = 0;
	unsigned char *pixels = stbi_load_from_memory(src, (int)src_size, &w, &h, &comp, 0);
	if (!pixels)
		return NULL;
	mc_blob_t blob = {0};
	int rc = stbi_write_png_to_func(blob_write, &blob, w, h, comp, pixels, w * comp);
	stbi_image_free(pixels);
	if (!rc) {
		free(blob.data);
		return NULL;
	}
	*out_size = blob.size;
	return blob.data;
}

/* Generate a 64x64 magenta/black checkerboard PNG to use as a visible
   placeholder when a referenced texture is missing on disk.  The hot-pink
   tint makes missing textures obvious in any viewer (matches the long-
   standing "missing material" convention used by Source / Quake tools). */
unsigned char *mc_image_make_missing_placeholder(size_t *out_size) {
	const int W = 64, H = 64, C = 4, CELL = 8;
	unsigned char *pixels = (unsigned char *)malloc((size_t)W * H * C);
	if (!pixels)
		return NULL;
	for (int y = 0; y < H; ++y) {
		for (int x = 0; x < W; ++x) {
			int cell = ((x / CELL) + (y / CELL)) & 1;
			unsigned char *p = &pixels[(y * W + x) * C];
			if (cell) {
				p[0] = 255; p[1] = 0; p[2] = 255; /* magenta */
			} else {
				p[0] = 0; p[1] = 0; p[2] = 0; /* black */
			}
			p[3] = 255;
		}
	}
	mc_blob_t blob = {0};
	int rc = stbi_write_png_to_func(blob_write, &blob, W, H, C, pixels, W * C);
	free(pixels);
	if (!rc) {
		free(blob.data);
		return NULL;
	}
	*out_size = blob.size;
	return blob.data;
}
