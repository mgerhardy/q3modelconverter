bl_info = {
    "name": "Quake 3 Model Converter",
    "author": "World of Padman Team",
    "version": (1, 0, 0),
    "blender": (3, 6, 0),
    "location": "File > Import/Export",
    "description": "Import/export MD3, IQM and MDR via modelconverter",
    "category": "Import-Export",
}

import bpy
import os
import subprocess
import tempfile
from bpy.props import StringProperty, BoolProperty, FloatProperty, IntProperty, EnumProperty
from bpy_extras.io_utils import ImportHelper, ExportHelper


def get_converter(context):
    prefs = context.preferences.addons[__name__].preferences
    p = prefs.converter_path
    if p and os.path.isfile(p):
        return p
    # Try PATH
    import shutil
    return shutil.which("modelconverter")


class Q3MCPreferences(bpy.types.AddonPreferences):
    bl_idname = __name__

    converter_path: StringProperty(
        name="modelconverter binary",
        subtype='FILE_PATH',
        default="",
        description="Path to the modelconverter executable (leave empty to search PATH)",
    )

    def draw(self, context):
        self.layout.prop(self, "converter_path")
        exe = get_converter(context)
        if exe:
            self.layout.label(text=f"Found: {exe}", icon='CHECKMARK')
        else:
            self.layout.label(text="modelconverter not found", icon='ERROR')


# ---- Import ----

class ImportQ3Model(bpy.types.Operator, ImportHelper):
    """Import a Quake 3 model (.md3/.iqm/.mdr) via modelconverter"""
    bl_idname = "import_scene.q3model"
    bl_label = "Import Q3 Model"
    bl_options = {'UNDO', 'PRESET'}

    filename_ext = ".md3"
    filter_glob: StringProperty(default="*.md3;*.iqm;*.mdr", options={'HIDDEN'})

    fps: FloatProperty(name="FPS hint", default=15.0, min=1.0, max=120.0,
                       description="Animation sample rate for glTF intermediate")

    def execute(self, context):
        exe = get_converter(context)
        if not exe:
            self.report({'ERROR'}, "modelconverter not found – set path in addon preferences")
            return {'CANCELLED'}

        with tempfile.TemporaryDirectory() as tmp:
            glb = os.path.join(tmp, "import.glb")
            cmd = [exe, "-i", self.filepath, "-o", glb, "--fps", str(self.fps)]
            try:
                subprocess.run(cmd, check=True, capture_output=True, text=True)
            except subprocess.CalledProcessError as e:
                self.report({'ERROR'}, f"modelconverter failed: {e.stderr}")
                return {'CANCELLED'}

            bpy.ops.import_scene.gltf(filepath=glb)

        self.report({'INFO'}, f"Imported {os.path.basename(self.filepath)}")
        return {'FINISHED'}


# ---- Export ----

class ExportQ3Model(bpy.types.Operator, ExportHelper):
    """Export to a Quake 3 model (.md3/.iqm/.mdr) via modelconverter"""
    bl_idname = "export_scene.q3model"
    bl_label = "Export Q3 Model"

    filename_ext = ".md3"
    filter_glob: StringProperty(default="*.md3;*.iqm;*.mdr;*.glb;*.gltf", options={'HIDDEN'})

    emit_skin: BoolProperty(name="Write .skin", default=False,
                            description="Also emit a Q3 .skin sidecar file")
    emit_shader: BoolProperty(name="Write .shader", default=False,
                              description="Also emit a Q3 .shader sidecar file")
    decimate: FloatProperty(name="Decimate ratio", default=1.0, min=0.01, max=1.0,
                            description="Keep this fraction of triangles (1.0 = no decimation)")
    gen_lods: IntProperty(name="Generate LODs", default=0, min=0, max=2,
                          description="Number of additional LOD levels to auto-generate")

    def execute(self, context):
        exe = get_converter(context)
        if not exe:
            self.report({'ERROR'}, "modelconverter not found – set path in addon preferences")
            return {'CANCELLED'}

        with tempfile.TemporaryDirectory() as tmp:
            glb = os.path.join(tmp, "export.glb")
            bpy.ops.export_scene.gltf(filepath=glb, export_format='GLB')

            cmd = [exe, "-i", glb, "-o", self.filepath]
            if self.emit_skin:
                cmd.append("--skin")
            if self.emit_shader:
                cmd.append("--shader")
            if self.decimate < 1.0:
                cmd += ["--decimate", str(self.decimate)]
            if self.gen_lods > 0:
                cmd += ["--gen-lods", str(self.gen_lods)]

            try:
                subprocess.run(cmd, check=True, capture_output=True, text=True)
            except subprocess.CalledProcessError as e:
                self.report({'ERROR'}, f"modelconverter failed: {e.stderr}")
                return {'CANCELLED'}

        self.report({'INFO'}, f"Exported {os.path.basename(self.filepath)}")
        return {'FINISHED'}


# ---- Registration ----

def menu_func_import(self, context):
    self.layout.operator(ImportQ3Model.bl_idname, text="Quake 3 Model (.md3/.iqm/.mdr)")

def menu_func_export(self, context):
    self.layout.operator(ExportQ3Model.bl_idname, text="Quake 3 Model (.md3/.iqm/.mdr)")

classes = (Q3MCPreferences, ImportQ3Model, ExportQ3Model)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)
    bpy.types.TOPBAR_MT_file_export.append(menu_func_export)

def unregister():
    bpy.types.TOPBAR_MT_file_export.remove(menu_func_export)
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)

if __name__ == "__main__":
    register()
