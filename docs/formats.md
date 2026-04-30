# Format references

External specs and source-of-truth references for every format
`modelconverter` reads or writes, plus a short note on every place we
have to be lossy.

## glTF 2.0

- Spec: <https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html>
- Reference implementation: <https://github.com/KhronosGroup/glTF-Sample-Renderer>
- Validator: <https://github.com/KhronosGroup/glTF-Validator>
- cgltf (parser/writer used here): <https://github.com/jkuhlmann/cgltf>

The CI smoke test (`scripts/roundtrip-test.sh`) runs every emitted
`.gltf` / `.glb` through `gltf_validator` and fails on any error or
warning.

## MD3

- Original docs (idTech 3): <https://icculus.org/homepages/phaethon/q3a/formats/md3format.html>
- Header in the WoP source: `worldofpadman/code/qcommon/qfiles.h`
- Hard limits enforced by the engine:
  - `MD3_MAX_LODS = 3`
  - `MD3_MAX_TRIANGLES = 8192` per surface
  - `MD3_MAX_VERTS = 4096` per surface
  - `MD3_MAX_SHADERS = 256` per surface
  - `MD3_MAX_FRAMES = 1024`
  - `MD3_MAX_SURFACES = 32` per model
  - `MD3_MAX_TAGS = 16` per frame

**Lossy**: positions are quantised to 1/64-unit grid, normals to 8-bit
lat/long. A glTF -> MD3 -> glTF round-trip introduces up to ~0.008 unit
position error and ~1.4° normal error per vertex.

## IQM

- Spec: <https://github.com/lsalzman/iqm/blob/master/iqm.txt>
- Reference tools (`iqm`, `iqmtool`): <https://github.com/lsalzman/iqm>
- Header in this repo: `src/iqm.h`

**Lossless** for everything we use: positions and normals stay at
`float`, joint TRS is exact, blend weights are quantised to uchar4
matching the spec.

## MDR

- ioquake3 reference impl (loader): see `code/renderercommon/tr_mdr.c`
  in any ioq3 fork.
- Header in this repo: `src/mc_mdr.c` (struct definitions inline).

**Lossy**:
- MDR's compressed-frame variant (`ofsFrames < 0`) is supported on read;
  compressed bones are decompressed to full 3×4 matrices transparently.
- MDR stores per-bone deltas in 16-bit fixed point; round-tripping a
  pose introduces small errors (sub-millimetre at typical scale).

## Quake 3 shader scripts

- Manual: <https://www.heppler.com/shader/shader/>
- Tutorial: id Software's `shader_manual.txt.
- Parser entry point: `src/mc_q3shader.c`.

We capture the raw stanza body verbatim into
`material.extras.q3_body_b64` on glTF write so a `.shader -> glTF ->
.shader` round-trip preserves every stage, `blendFunc`, `tcMod`,
`rgbGen` exactly. The parsed metadata (cull, surfaceparm, alphaMode,
companion normal/emissive maps) is also stamped onto the material in
the standard glTF slots so a tool that ignores extras still gets a
sensible PBR approximation.

## animation.cfg

- Per-model file accompanying player MD3 bundles
  (`models/players/<name>/animation.cfg`).
- Format reference: id's `animation.cfg` in any q3 player asset and
  the parser in `worldofpadman/code/cgame/cg_players.c`.

We round-trip the full file through `extras.q3_animation_cfg` on the
scene plus a sidecar `animation.cfg` next to the player bundle. Out-of-
range firstFrame / numFrames / loopFrames / fps values are warned about
on load but kept verbatim.
