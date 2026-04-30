# Converting Models with `modelconverter` and `gltfpack`

This tutorial explains how to use the `modelconverter` tool that ships with
the World of Padman source tree to move assets between modern glTF / GLB
files and the classic Quake 3 `MD3`, `IQM` and `MDR` formats. It also shows how
to combine `modelconverter` with [gltfpack] for a clean, optimised content
pipeline.

[gltfpack]: https://github.com/zeux/meshoptimizer/blob/master/gltf/README.md

---

## Why bother?

Quake 3's native model formats are still the only ones the WoP engine can
load at runtime:

- **MD3** - vertex-animated meshes with optional `tag_*` attachment
  points. Used for player models, weapons, items and most map objects.
- **IQM** - skeletal-animated meshes (or static meshes when no skeleton is
  present).
- **MDR** - the legacy Team Arena / EliteForce skeletal format.

Modern DCC tools (Blender, Maya, 3ds Max, Houdini, Substance) all export
to glTF 2.0 with a single click. `modelconverter` bridges the two worlds
in both directions:

```
.gltf / .glb  <------------------>  .md3 / .iqm / .mdr
                 modelconverter
```

`gltfpack`, part of the [meshoptimizer] project, is the recommended
preprocessing step for any glTF that is destined for an MD3 export: it
removes unused data, quantises vertex attributes and shrinks textures so
the resulting MD3 stays well below the engine's hard limits
(4096 vertices and 8192 triangles per surface).

[meshoptimizer]: https://github.com/zeux/meshoptimizer

---

## Format support matrix

What every direction of the conversion preserves today. **L** = lossless,
**+** = supported (with the caveat in the notes), **-** = not applicable
to that format, **!** = supported but lossy by format design, **x** = not
yet implemented.

| Feature                       | MD3 read | MD3 write | IQM read | IQM write | MDR read | MDR write | glTF read | glTF write |
|-------------------------------|:--------:|:---------:|:--------:|:---------:|:--------:|:---------:|:---------:|:----------:|
| Static meshes                 |    L     |     L     |    L     |     L     |    L     |     L     |     L     |     L      |
| Per-vertex morph animation    |    L     |     L     |    -     |     -     |    -     |     -     |    L (1)  |    L (1)   |
| Skeletal animation            |    -     |     -     |    L     |     L     |    L     |     L     |     L     |     L      |
| Tags (per-frame TRS)          |    L     |     L     |    -     |     -     |    L     |     L     |    L (2)  |    L (2)   |
| Multi-LOD                     |    L     |     L     |    -     |     -     |    L     |     L     |    L (3)  |    L (3)   |
| Skins (`.skin` sidecar)       |    L     |     L     |    -     |     -     |    -     |     -     |    L (4)  |    L (4)   |
| Material baseColor            |    !     |     !     |    !     |     !     |    !     |     !     |     L     |     L      |
| Material normal               |    +     |     +     |    +     |     +     |    +     |     +     |     L     |     L      |
| Material metallic/roughness   |    -     |     -     |    -     |     -     |    -     |     -     |     L     |     L      |
| Material occlusion / emissive |    -     |     -     |    -     |     -     |    -     |     -     |     L     |     L      |
| alphaMode + alphaCutoff       |    !     |     !     |    !     |     !     |    !     |     !     |     L     |     L      |
| doubleSided                   |    !     |     !     |    !     |     !     |    !     |     !     |     L     |     L      |
| Q3 `.shader` round-trip       |    +     |     +     |    +     |     +     |    +     |     +     |    L (5)  |    L (5)   |
| `animation.cfg` (player)      |   L (6)  |   L (6)   |    -     |     -     |    -     |     -     |    L (6)  |    L (6)   |

Notes:

1. glTF carries vertex morphs as morph targets (one target per non-bind
   frame). Importers that ignore morph weights will see only the bind
   pose; the bake to MD3 still reproduces every frame.
2. Tags become nodes named `tag_*` with per-frame `cgltf_animation`
   tracks driving TRS.
3. LOD chains are encoded as `q3_lod` extras on the surface / tag nodes
   so a `glb -> dir -> glb` round-trip preserves all slots.
4. `.skin` variants become sibling glTF files when exporting a player
   bundle (`--player`); a glTF that already carries multiple textures
   per part can re-emit a `.skin` file on its way back to MD3.
5. Each surface that matched a `.shader` stanza captures the raw stanza
   body into `material.extras.q3_body_b64` (base64). On glTF -> MD3 the
   `.shader` writer re-emits it verbatim, so every stage / `blendFunc` /
   `tcMod` / `rgbGen` survives losslessly.
6. `animation.cfg` is preserved as a JSON block in the scene `extras`
   plus a sidecar file when `--player` is used. Out-of-range firstFrame /
   numFrames / loopFrames / fps values are warned about but kept.

Lossy points worth knowing about:

- **MD3 vertex quantisation:** positions snap to a 1/64-unit grid and
  normals to 8-bit lat/long. Round-tripping through MD3 will introduce
  position errors up to ~0.008 units and normal errors up to ~1.4°.
- **glTF -> MDR animation sampling:** glTF animation tracks are sampled
  at the model `--fps` (default 15). Cubic-spline tracks are evaluated
  exactly at sample points; sub-sample motion is lost.
- **MDR compressed-frame variant** (`ofsFrames < 0`): supported on read.
  Compressed bones are decompressed to full 3×4 matrices transparently.
- **PK3 (zip) shader auto-discovery:** `--shader-path` and `--asset-root`
  accept `.pk3` files directly; shader stanzas are read from the archive
  without manual extraction.

For a deeper dive into each format (specs, reference implementations,
hard limits, exactly where we have to be lossy), see
[docs/formats.md](docs/formats.md).

---

## Naming conventions

The reader infers a lot from node and surface names so author-side glTFs
do not need to carry custom extras:

| Pattern                               | Meaning                                                |
|---------------------------------------|--------------------------------------------------------|
| `tag_*`                               | Tag (no mesh, axes follow node TRS)                    |
| `head*` / `h_*`                       | Player **head** part                                   |
| `upper*` / `torso*` / `u_*`           | Player **upper** part                                  |
| `lower*` / `legs*` / `l_*`            | Player **lower** part                                  |
| `<name>_lod<N>` or `<name>_<N>`       | Additional LOD level (N >= 1)                          |
| `<basename>_n.<ext>`                  | Companion **normal** map next to a base color texture  |
| `<basename>_nh.<ext>`                 | Companion **normal+height** map (opengl2-style)        |

A glTF that touches all three of `head` / `upper` / `lower` is auto-promoted
to a player bundle on export. Extras (`q3_part`, `q3_lod`, `q3_tag`,
`q3_shader`, `q3_normal`, `q3_normalheight`, `q3_body_b64`) take precedence
over name inference when present.

---

## Building `modelconverter`

The tool is built as a standalone CMake project. From the repository root:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

The resulting binary lives at
`build/modelconverter`. There are no runtime
dependencies; it links only against the C standard library and `libm`.

The source lives in `src/` and embeds the single-file
[`cgltf`] / [`cgltf_write`] headers as well `stb` headers for image support,
so adding the tool to your own out-of-tree build should be easy enough.

[`cgltf`]: https://github.com/jkuhlmann/cgltf

---

## Command-line reference

Format detection happens by file extension. If you need to inspect what
the tool *thinks* is in a file, run `--info` first:

```bash
modelconverter --info -i upper.md3
```

The same operations are also available as bare positional subcommands
for people who prefer that style:

```bash
modelconverter info upper.md3
modelconverter validate upper.md3
modelconverter convert upper.md3 upper.glb
modelconverter player models/players/fatpad/ fatpad.glb
```

Both forms work interchangeably. `--quiet` silences info/debug output
(errors only); `--verbose` enables debug-level tracing. `--validate`
runs format-limit and sanity checks on the input and exits non-zero if
anything looks off (no output is written).

```text
file: upper.md3
  surfaces : 1
  frames   : 155
  tags     : 3
  [surf 0] name='u_torso' shader='models/players/fatpad/fatpadtorsoskin.jpg' verts=390 tris=476
  [tag  0] name='tag_head' origin=(3.874 0.095 12.160)
  [tag  1] name='tag_weapon' origin=(-4.336 -5.077 21.990)
  [tag  2] name='tag_torso' origin=(0.000 -0.000 0.000)
```

---

## Workflow A: glTF/GLB -> MD3

This is the most common path: an artist hands you a `.glb` exported from
Blender, and you want a Quake 3 `.md3`.

### 1. Optimise with `gltfpack`

`gltfpack` is a one-shot optimiser. Install it with the project's release
binaries or via `npm install -g gltfpack`. The most useful flags for our
purposes are:

| Flag | What it does |
|------|--------------|
| `-cc` | Compress vertex data (useful if the asset will stay glTF). |
| `-tc` | Re-encode textures to KTX2 (skip when targeting MD3). |
| `-si <N>` | Simplify the mesh down to N% of its original triangle count. |
| `-noq` | **Disable quantisation.** Required when the output will be re-read by `modelconverter` because MD3 needs plain `float` POSITION/NORMAL accessors. |
| `-tr` | Strip animation tracks unrelated to the bind pose. |
| `-ke` | Keep extras (preserves our `q3_tag` markers). |

Example - shrink to 60 % of the original triangle count and keep all
non-mesh nodes intact:

```bash
gltfpack -i player.glb -o player_opt.glb -si 0.6 -noq -ke
```

> **Important:** if you let `gltfpack` quantise positions (`-vp` etc.)
> the result will be valid glTF but `modelconverter` will read garbage,
> because MD3 itself stores 16-bit positions and we already perform the
> per-frame quantisation. Either skip quantisation (`-noq`) or feed the
> result back through a tool that dequantises before MD3 export.

### 2. Convert to MD3

```bash
modelconverter -v -i player_opt.glb -o models/players/fatpad/upper.md3 --skin
```

What happens:

- Every glTF *primitive* becomes one MD3 surface. The surface name is
  taken from the owning node (or the mesh name as a fallback).
- glTF nodes whose name starts with `tag_` and that carry no mesh become
  MD3 tags. The node's local axes (forward = +X, left = +Y, up = +Z)
  are stored as the tag axes - same convention as idTech 3.
- Material `baseColorTexture` paths become MD3 shader names (with the
  file extension stripped to match Quake 3 conventions).
- `--skin` writes a sibling `.skin` file with one `surface,shader` line
  per surface, ready for placement next to the model.
- `--shader` writes a sibling `.shader` file generated from the glTF
  materials. One stanza per unique shader name is emitted, with `cull disable`
  for `doubleSided` materials, `surfaceparm trans` plus
  `blendFunc blend` or `alphaFunc GE128` for translucent / masked
  alpha modes, and the base color image as the first stage's `map`.
  Normal, emissive and metallic-roughness textures are added as
  opengl2-style stages so the asset round-trips through the renderer
  that supports them.
- `--shader-in <path>` parses an existing `.shader` file *before* writing
  the output and uses the matching stanza to override each surface's
  texture, cull mode, surfaceparm and blend mode. This is how you bind
  a hand-authored `.shader` to a glTF that doesn't carry the right
  textures yet.

When loading a glTF, the converter automatically looks for `_n` and
`_nh` companion textures next to each material's base color image.
Found companions are copied alongside the output and referenced
in the generated `.shader` as `normalMap`. Their URIs are also stamped
into the glTF material `extras` as `q3_normal` / `q3_normalheight` so a
glb-only round-trip preserves them.

### 3. Verify in-engine

Drop the resulting `.md3` plus its texture(s) and `.skin` files into the
appropriate `.pk3dir` and start WoP. Use `r_showtris 1` and
`r_showmodelbounds 1` in the console to verify the mesh and tag layout.

---

## Workflow B: MD3 / IQM / MDR -> glTF / GLB

Useful for porting old assets into modern DCC tools, batch-inspecting
content, or producing previews for a mod's website.

```bash
modelconverter -v -i upper.md3 -o upper.glb
```

Multi-frame MD3 vertex animation is encoded as **glTF morph targets**
(one target per additional frame, on top of the bind pose). Most modern
viewers (Blender, Three.js, Babylon.js, glTF-Sample-Viewer) can scrub
through these targets directly.

Tags become glTF nodes named `tag_<name>`, with `extras = { "q3_tag": true }`
so the tool can identify them again on a later round-trip.

---

## Workflow C: Round-tripping through `gltfpack`

For published assets you usually want the MD3 *and* a clean glTF the
community can edit. The full pipeline looks like this:

```bash
# 1. Source asset (from the modeller).
artist_export.glb

# 2. Inspect it.
modelconverter --info -i artist_export.glb

# 3. Optimise without quantisation.
gltfpack -i artist_export.glb -o opt.glb -si 0.8 -noq -ke

# 4. Produce the engine-ready MD3 and a sibling .skin file.
modelconverter -i opt.glb -o models/mapobjects/foo.md3 --skin

# 5. (Optional) ship a preview-friendly, fully optimised glTF too.
gltfpack -i opt.glb -o foo_preview.glb -tc -cc
```

The fifth step is **only** for the public preview - never feed a
quantised / KTX2 file back into `modelconverter`.

---

## IQM and MDR skeletal support

`modelconverter` reads and writes **fully skeletal** IQM and MDR files
when the source model carries a skeleton (joints, per-frame TRS poses,
per-vertex blend indices and weights). The skeleton, animation entries
and skinning data round-trip losslessly through IQM, MDR and glTF.
When the source has no skeleton (e.g. a plain MD3 or a static glTF)
the writers fall back to a static single-frame output — enough for map
objects, items and pickups.

Reading IQM preserves the full joint hierarchy, per-frame channel
data and per-vertex BLENDINDEXES / BLENDWEIGHTS. In addition every
frame is evaluated against the skeleton to produce baked per-frame
xyz/normal arrays so vertex-animated consumers (MD3, glTF morph
targets) can replay the animation too.

Reading MDR preserves the bone matrices, per-vertex weights (up to 4
influences) and per-frame tag transforms. The skeleton is mirrored
into the generic representation so it survives a round-trip through
IQM or glTF. Only LOD 0 (highest detail) is read; the compressed MDR
variant (`mdrCompFrame_t`) is decompressed transparently on read.

---

## Troubleshooting

- **`md3: surface 'X' has too many verts (N > 4096)`** - run the source
  asset through `gltfpack -si <ratio>` to reduce triangles, then re-run
  `modelconverter`.
- **Textures appear as the default cyan checkerboard in-engine** - check
  the shader name printed by `--info`. It must match a `textures/...`
  path inside one of your `.pk3` / `.pk3dir` files. Materials without a
  baseColor texture fall back to the *material name* as the shader name.
- **Tags moved or rotated unexpectedly after a round-trip** - confirm
  the source glTF stores tag nodes with no mesh and a name starting
  with `tag_`. Anything attached as a child of a mesh is treated as
  geometry, not a tag.
- **Quantised glTF reads as exploded or zero-sized geometry** - run
  `gltfpack` again with `-noq`, or use any glTF dequantiser before
  feeding it to `modelconverter`.

---

## See also

- [`gltfpack` README](https://github.com/zeux/meshoptimizer/blob/master/gltf/README.md) -
  full list of optimisation flags.
- [`cgltf` upstream](https://github.com/jkuhlmann/cgltf) - the parser /
  writer the converter is built on.

## AI

This project was created with the help of AI
