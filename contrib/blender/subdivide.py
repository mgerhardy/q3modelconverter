# blender -b -P subdivide.py -- input.glb output.glb

import bpy
import sys

argv = sys.argv
argv = argv[argv.index("--") + 1:]
input_file, output_file = argv

bpy.ops.import_scene.gltf(filepath=input_file)

for obj in bpy.context.scene.objects:
    if obj.type == 'MESH':
        mod = obj.modifiers.new(name="Subdiv", type='SUBSURF')
        mod.levels = 2
        mod.render_levels = 2
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.modifier_apply(modifier=mod.name)

bpy.ops.export_scene.gltf(filepath=output_file)
