# CLI Reference

## Usage

```
modelconverter [options] -i <input> -o <output>
modelconverter --info -i <file>
modelconverter --validate -i <file>
modelconverter --player -i <dir> -o <output>
```

## Options

| Flag | Description |
|------|-------------|
| `-i, --input <path>` | Input model or player directory |
| `-o, --output <path>` | Output model or directory |
| `--fps <N>` | glTF animation sample rate (default 15) |
| `--skin` | Emit a `.skin` sidecar file |
| `--shader` | Emit a `.shader` sidecar file |
| `--shader-in <path>` | Apply an existing `.shader` file to the model |
| `--skin-in <path>` | Apply an existing `.skin` file to the model |
| `--no-auto-skin` | Don't auto-discover `.skin` files |
| `--no-auto-shader` | Don't auto-discover `.shader` files |
| `--player` | Force player-bundle mode |
| `--subdivide <N>` | Apply N iterations of Loop subdivision |
| `--decimate <R>` | QEM decimation keeping R×100% of triangles |
| `--gen-lods [N]` | Auto-generate N additional LOD levels (default 2) |
| `--gen-lod-ratios <r1,r2,...>` | Triangle-keep ratios per LOD |
| `--asset-root <dir>` | Add a texture search directory or `.pk3` file (repeatable) |
| `--shader-path <dir>` | Add a shader search directory or `.pk3` file (repeatable) |
| `--shader-depth <N>` | Max recursion depth for shader discovery (default 6) |
| `--info` | Print info and exit |
| `--validate` | Run checks and exit non-zero on issues |
| `-v, --verbose` | Verbose output |
| `-q, --quiet` | Errors only |
| `-h, --help` | Show help |

## Supported formats

| Extension | Read | Write |
|-----------|------|-------|
| `.gltf` | ✓ | ✓ |
| `.glb` | ✓ | ✓ |
| `.md3` | ✓ | ✓ |
| `.iqm` | ✓ | ✓ |
| `.mdr` | ✓ (including compressed frames) | ✓ |

## Examples

```bash
# Batch convert all MD3s in a directory to GLB
for f in models/*.md3; do
  modelconverter -i "$f" -o "${f%.md3}.glb"
done

# Generate LODs for a weapon model
modelconverter -i weapon.glb -o weapon.md3 --gen-lods 2

# Decimate to 50% and export
modelconverter -i highpoly.glb -o lowpoly.md3 --decimate 0.5
```
