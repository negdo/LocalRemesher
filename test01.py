import bpy

# Get the active object in the scene
obj = bpy.context.active_object

# Get the active mesh
if obj.mode == 'EDIT':
    mesh = obj.data
else:
    mesh = obj.data.copy()
    mesh.transform(obj.matrix_world)

# Get the selected faces
selected_faces = [f for f in mesh.polygons if f.select]

mesh_loops = mesh.loops
selected_loop_edges = []
print()
for face in selected_faces:
    for loop_index in face.loop_indices:
        loop = mesh_loops[loop_index]
        selected_loop_edges.append(loop.index)

print(selected_loop_edges)

selected_loops = []


for loop in mesh_loops:
    if loop.edge_index in selected_loop_edges:
        selected_loops.append(loop.index)

print(selected_loops)

for loop in mesh_loops:
    if loop.index == 28:
        print(loop.edge_index)


# Delete the selected faces
# bpy.ops.mesh.delete(type='FACE')
