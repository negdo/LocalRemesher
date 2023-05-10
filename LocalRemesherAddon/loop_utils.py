import bpy
import bmesh
import numpy as np
import mathutils
import sys

sys.path.append("C:/Users/miham/Documents/3d/blender/scripts/local remesh/LocalRemesher/rewrite")
from Directed_edge import *

# select next vertex in loop
def select_next(current_edge, vertex, selected_edges, illegal_edges):
    # check if vertex has only one edge
    if len(vertex.link_edges)  == 1:
        return selected_edges, vertex

    # get potential edges
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
            return select_next(new_edges[0], new_edges[0].other_vert(vertex), selected_edges, illegal_edges)

    # if there are two edges, find the most parralel one
    elif len(new_edges) >= 2:
        # get angles between edges
        angles = [abs(np.dot((edge.verts[0].co - edge.verts[1].co)/np.linalg.norm(edge.verts[0].co - edge.verts[1].co), (current_edge.verts[0].co - current_edge.verts[1].co)/np.linalg.norm(current_edge.verts[0].co - current_edge.verts[1].co))) for edge in new_edges]

        index = np.argmax(angles)

        if new_edges[index] not in selected_edges and new_edges[index] not in illegal_edges:
            selected_edges.append(new_edges[index])

            # continue with the next vertex recursively
            return select_next(new_edges[index], new_edges[index].other_vert(vertex), selected_edges, illegal_edges)

    # check, if any edges are close to parralel
    elif len(new_edges) == 0:
        # get angles between edges
        angles = [abs(np.dot((edge.verts[0].co - edge.verts[1].co)/np.linalg.norm(edge.verts[0].co - edge.verts[1].co), (current_edge.verts[0].co - current_edge.verts[1].co)/np.linalg.norm(current_edge.verts[0].co - current_edge.verts[1].co))) for edge in edges]

        index = np.argmax(angles)

        if edges[index] not in selected_edges and edges[index] not in illegal_edges:
            if angles[index] > 0.9:
                selected_edges.append(edges[index])

                # continue with the next vertex recursively
                return select_next(edges[index], edges[index].other_vert(vertex), selected_edges, illegal_edges)
    
    return selected_edges, vertex


# get geometry edge loop starting from edge
def get_loop(edge, illegal_edges):
    # go both directions
    selected_edges_left, last_vert = select_next(edge, edge.verts[0], [edge], illegal_edges)
    selected_edges_right, last_vert_r = select_next(edge, edge.verts[1], [edge], illegal_edges)

    # check if loop is at least 3 edges long and not continous
    if len(selected_edges_left) == 1 and len(selected_edges_right) > 2:
        return Directed_edge(selected_edges_right[-1], last_vert_r)
    elif len(selected_edges_right) == 1 and len(selected_edges_left) > 2:
        return Directed_edge(selected_edges_left[-1], last_vert)
    else:
        return None



