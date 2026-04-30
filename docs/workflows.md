# Workflow Recipes

## Blender → MD3

```bash
# 1. Export from Blender as GLB (File > Export > glTF)
# 2. Optionally optimise with gltfpack
gltfpack -i player.glb -o player_opt.glb -si 0.8 -noq -ke

# 3. Convert to MD3 with skin and shader sidecars
modelconverter -i player_opt.glb -o upper.md3 --skin --shader
```

## Player bundle round-trip

```bash
# Export a full player directory to a single GLB
modelconverter --player -i models/players/padman/ -o padman.glb

# Edit in Blender, then split back to MD3s
modelconverter --player -i padman_edited.glb -o models/players/padman_new/
```

## Batch conversion

```bash
# Convert every MD3 in a mod to GLB for inspection
find wop/ -name '*.md3' | while read f; do
  out="${f%.md3}.glb"
  modelconverter -i "$f" -o "$out" 2>/dev/null && echo "OK $f"
done
```

## LOD generation

```bash
# Generate 2 LOD levels (50% and 25% triangle density)
modelconverter -i weapon.glb -o weapon.md3 --gen-lods 2

# Custom ratios
modelconverter -i weapon.glb -o weapon.md3 \
  --gen-lods 3 --gen-lod-ratios 0.6,0.3,0.1
```

## Shader authoring

```bash
# Apply a hand-authored .shader to a model
modelconverter -i model.glb -o model.md3 --shader-in custom.shader

# Auto-discover shaders from a pk3
modelconverter -i model.glb -o model.md3 --shader-path baseq3/pak0.pk3
```

## CI validation

```bash
# In a GitHub Actions workflow:
modelconverter --validate -i models/players/padman/upper.md3
# Exits non-zero if any format limit is exceeded
```
