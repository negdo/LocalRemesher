import numpy as np
import bmesh
import bpy

def selectNext(current_edge, vertex, selected_edges, illegal_edges):
    
    # check if vertex has 4 edges
    if len(vertex.link_edges)  == 1:
        return selected_edges

    # get the 3 potential edges
    edges = [edge for edge in vertex.link_edges if edge != current_edge]

    # get faces connected to current edge
    faces = [face for face in current_edge.link_faces]
    bad_edges = [edge for face in faces for edge in face.edges]

    # get edges that do not belong to any of the faces around the current edge
    new_edges = [edge for edge in edges if edge not in bad_edges]

    # if there is only one edge, select it
    if len(new_edges) == 1:
        # check if edge is already selected
        if new_edges[0] not in selected_edges and new_edges[0] not in illegal_edges:
            selected_edges.append(new_edges[0])

            # continue with the next vertex recursively
            selectNext(new_edges[0], new_edges[0].other_vert(vertex), selected_edges, illegal_edges)

    # if there are two edges, find the most parralel one
    elif len(new_edges) >= 2:
        # get angles between edges
        angles = [abs(np.dot((edge.verts[0].co - edge.verts[1].co)/np.linalg.norm(edge.verts[0].co - edge.verts[1].co), (current_edge.verts[0].co - current_edge.verts[1].co)/np.linalg.norm(current_edge.verts[0].co - current_edge.verts[1].co))) for edge in new_edges]

        index = np.argmax(angles)

        if new_edges[index] not in selected_edges and new_edges[index] not in illegal_edges:
            selected_edges.append(new_edges[index])

            # continue with the next vertex recursively
            selectNext(new_edges[index], new_edges[index].other_vert(vertex), selected_edges, illegal_edges)

    # check, if any edges are close to parralel
    elif len(new_edges) == 0:
        # get angles between edges
        angles = [abs(np.dot((edge.verts[0].co - edge.verts[1].co)/np.linalg.norm(edge.verts[0].co - edge.verts[1].co), (current_edge.verts[0].co - current_edge.verts[1].co)/np.linalg.norm(current_edge.verts[0].co - current_edge.verts[1].co))) for edge in edges]

        index = np.argmax(angles)

        if edges[index] not in selected_edges and edges[index] not in illegal_edges:
            if angles[index] > 0.9:
                selected_edges.append(edges[index])

                # continue with the next vertex recursively
                selectNext(edges[index], edges[index].other_vert(vertex), selected_edges, illegal_edges)

        

    return selected_edges


def get_loop(edge, illegal_edges, semi_illegal_edges):
    # go both directions
    print("")
    print("edge: ", edge.index)
    selected_edges_left = selectNext(edge, edge.verts[0], [edge], illegal_edges)
    print("left:", len(selected_edges_left))
    selected_edges_right = selectNext(edge, edge.verts[1], [edge], illegal_edges)
    print("right:", len(selected_edges_right))

    if len(selected_edges_left) == 1:
        return selected_edges_right
    elif len(selected_edges_right) == 1:
        return selected_edges_left

    return [edge]



# set mode to object mode and init bmesh
bpy.ops.object.mode_set(mode='OBJECT')
me = bpy.context.object.data
bm = bmesh.new()
bm.from_mesh(me)

# get all selected edges
selected_edges = [edge for edge in bm.edges if edge.select]

# get loop
loop = get_loop(selected_edges[0], [], [])

# select all edges in the loop
for edge in loop:
    edge.select = True


# write the bmesh back to the mesh
bm.to_mesh(me)
bpy.ops.object.mode_set(mode='EDIT')