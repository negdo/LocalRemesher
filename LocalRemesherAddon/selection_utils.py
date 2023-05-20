import bpy
import bmesh
import numpy as np
import mathutils
import sys

sys.path.append("C:/Users/miham/Documents/3d/blender/scripts/local remesh/LocalRemesher/rewrite")
from .Directed_edge import *

# returns selected faces and their vertices
def get_selected(bm):
    faces_selected = [face for face in bm.faces if face.select]
    verts_selected = list(set(vert for face in faces_selected for vert in face.verts))

    return faces_selected, verts_selected


def get_starting(faces_selected, verts_selected):
    edges_of_faces = list(set(edge for face in faces_selected for edge in face.edges))
    # vertices that are connected to at least one face that is not selected
    starting_verts = [vert for vert in verts_selected if len([face for face in vert.link_faces if not face.select]) > 0]

    # edges that are connected to at least one outside face and few other stuff
    starting_edges = []

    for vert in starting_verts:
        edges_of_vert = [edge for edge in vert.link_edges if len([face for face in edge.link_faces if not face.select]) > 0]

        # limit number of edges from one vertex, to have squarish mesh
        if len(edges_of_vert) == 1:
            starting_edges.append(Directed_edge(edges_of_vert[0], vert))
        elif len(edges_of_vert) == 2:
            starting_edges.append(Directed_edge(edges_of_vert[0], vert))
            #starting_edges.append(Directed_edge(edges_of_vert[1], vert))
        elif len(edges_of_vert) >= 3:
            # to get more squarish mesh, only use edge that has the most outside faces connected to it
            # get number of outside faces connected to each edge
            num_outside_faces = [len([face for face in edge.link_faces if not face.select]) for edge in edges_of_vert]

            # get index of edge with most outside faces
            index = np.argmax(num_outside_faces)
            starting_edges.append(Directed_edge(edges_of_vert[index], vert))

            if len(edges_of_vert) >= 4:
                pass
                #num_outside_faces.pop(index)
                #edges_of_vert.pop(index)
                #index = np.argmax(num_outside_faces)

                #starting_edges.append(Directed_edge(edges_of_vert[index], vert))


    # illegal edges are inside the selected faces and do not connect to any outside face
    illegal_edges = [edge for edge in edges_of_faces if len([face for face in edge.link_faces if not face.select]) == 0]

    # average length of starting edges
    avg_length = np.mean([edge.edge.calc_length() for edge in starting_edges])

    return starting_edges, illegal_edges, avg_length


def sort_vertices(vertices):
    # sort vertices by location, first by x, then by y, then by z
    vertices.sort(key=lambda vert: vert.co.z)
    vertices.sort(key=lambda vert: vert.co.y)
    vertices.sort(key=lambda vert: vert.co.x)

    return vertices


def get_triangles(edges):
    # go over all new edges
    # check all linked edges of each vertex of the edge
    # if sets of vertices of these edges have same vertex, then this is a triangle
    triangles = set()
    existing_triangles = set()
    edges.sort(key=lambda edge: edge.index)

    for edge in edges:
        
        for edge_1 in edge.verts[0].link_edges:
            for edge_2 in edge.verts[1].link_edges:
                if edge_1.other_vert(edge.verts[0]) == edge_2.other_vert(edge.verts[1]):
                    # found enclosed triangle

                    # check if there is already a face with these vertices
                    faces_0 = set(face for face in edge.link_faces)
                    faces_1 = set(face for face in edge_1.link_faces)
                    faces_2 = set(face for face in edge_2.link_faces)
                    common_faces = faces_0.intersection(faces_1, faces_2)

                    triangle = tuple(sort_vertices([edge.verts[0], edge.verts[1], edge_2.other_vert(edge.verts[1])]))

                    # get intersection of faces, if there is none, we found empty triangle
                    if len(common_faces) == 0:
                        triangles.add(triangle)
                    elif len(common_faces) == 1:
                        existing_triangles.add(common_faces.pop())
                    else:
                        print("Error: more than one face with same vertices")




    return list(triangles), list(existing_triangles)





        


        



                






