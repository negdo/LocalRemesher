import bpy


class LocalRemesherPanel(bpy.types.Panel):
    bl_idname = "LOCAL_REMESHER_PT_Panel"
    bl_label = "Local Remesher"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "LocalRemesher"
    bl_context = "mesh_edit"

    def draw(self, context):
        layout = self.layout
        layout.operator("localremesher.remesh_grid", text="Grid Remesher")
        layout.operator("localremesher.remesh_particle", text="Particle Remesher")
