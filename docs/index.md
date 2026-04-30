# modelconverter

A command-line tool for converting between glTF/GLB and Quake 3 model
formats (MD3, IQM, MDR). Built for the [World of Padman](https://worldofpadman.net)
project but works with any idTech 3 based game.

## Features

- **Bidirectional conversion** between glTF 2.0 and MD3/IQM/MDR
- **Skeletal animation** round-trip through IQM, MDR and glTF skins
- **Player bundle** support (head/upper/lower MD3 + animation.cfg + skins)
- **Shader round-trip** — Q3 `.shader` stanzas survive a glTF round-trip via base64 extras
- **Mesh processing** — Loop subdivision, QEM decimation, auto-LOD generation
- **PK3 support** — reads `.pk3` archives directly for shader auto-discovery
- **Validation** — `--validate` checks format limits and exits non-zero on issues
- Zero runtime dependencies (libc + libm only)

## Quick start

```bash
# Convert a glTF to MD3
modelconverter -i player.glb -o upper.md3

# Inspect a model
modelconverter --info -i upper.md3

# Full player bundle round-trip
modelconverter --player -i models/players/padman/ -o padman.glb
modelconverter --player -i padman.glb -o models/players/padman_new/
```

See the [Getting Started](getting-started.md) guide for installation
and the [CLI Reference](cli-reference.md) for all options.
