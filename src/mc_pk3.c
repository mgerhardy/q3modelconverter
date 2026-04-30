/*
===========================================================================
modelconverter - PK3 (zip) archive reading support.

Allows --shader-path, --asset-root and -i to accept .pk3 files directly.
Uses miniz for zip decompression.
===========================================================================
*/

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-Wmissing-prototypes"
#pragma GCC diagnostic ignored "-Wstrict-prototypes"
#endif
#include "miniz/miniz.h"
#include "miniz/miniz_zip.h"
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include "mc_common.h"
#include <string.h>

int mc_pk3_is_pk3(const char *path) {
	if (!path) return 0;
	size_t len = strlen(path);
	return (len >= 4 && strcasecmp(path + len - 4, ".pk3") == 0);
}

/* Read a single file from a PK3 archive.  Returns a malloc'd buffer
   (caller frees) and writes the size to *out_size.  Returns NULL if
   the entry is not found or decompression fails. */
unsigned char *mc_pk3_read_entry(const char *pk3_path, const char *entry_name, size_t *out_size) {
	mz_zip_archive zip;
	memset(&zip, 0, sizeof(zip));
	if (!mz_zip_reader_init_file(&zip, pk3_path, 0))
		return NULL;

	int idx = mz_zip_reader_locate_file(&zip, entry_name, NULL, 0);
	if (idx < 0) {
		mz_zip_reader_end(&zip);
		return NULL;
	}

	size_t sz = 0;
	void *data = mz_zip_reader_extract_to_heap(&zip, (mz_uint)idx, &sz, 0);
	mz_zip_reader_end(&zip);

	if (!data) return NULL;
	if (out_size) *out_size = sz;
	return (unsigned char *)data;
}

/* List all files in a PK3 matching a suffix (e.g. ".shader").
   Calls the callback for each match with the entry name.
   Returns the number of matches. */
int mc_pk3_for_each(const char *pk3_path, const char *suffix,
                    void (*cb)(const char *pk3, const char *entry, void *user), void *user) {
	mz_zip_archive zip;
	memset(&zip, 0, sizeof(zip));
	if (!mz_zip_reader_init_file(&zip, pk3_path, 0))
		return 0;

	size_t slen = suffix ? strlen(suffix) : 0;
	int count = 0;
	mz_uint n = mz_zip_reader_get_num_files(&zip);
	for (mz_uint i = 0; i < n; ++i) {
		char name[512];
		if (!mz_zip_reader_get_filename(&zip, i, name, sizeof(name)))
			continue;
		if (mz_zip_reader_is_file_a_directory(&zip, i))
			continue;
		if (slen > 0) {
			size_t nlen = strlen(name);
			if (nlen < slen || strcasecmp(name + nlen - slen, suffix) != 0)
				continue;
		}
		if (cb) cb(pk3_path, name, user);
		++count;
	}

	mz_zip_reader_end(&zip);
	return count;
}
