import bpy
import bmesh
import numpy as np
import mathutils
import sys

sys.path.append("C:/Users/miham/Documents/3d/blender/scripts/local remesh/LocalRemesher/rewrite")
from Directed_edge import *

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


def get_vertex_path(edge_path):
    # get vertex path from edge path
    vertex_path = [edge_path[0].verts[0]]
    for edge in edge_path:
        vertex_path.append(edge.other_vert(vertex_path[-1]))
        
        # check if index is -1
        if vertex_path[-1].index == -1:
            print("Error: vertex index is -1")

    vertex_path = list(set(vertex_path))

    # sort path by index, but vertices have to be connected
    vertex_path.sort(key=lambda vert: vert.index)
    sorted_path = [vertex_path.pop(0)]
    for i in range(len(vertex_path)):
        linked_verts = [edge.other_vert(sorted_path[-1]) for edge in sorted_path[-1].link_edges]
        for j in range(len(vertex_path)):
            if vertex_path[j] in linked_verts:
                sorted_path.append(vertex_path.pop(j))
                break
        
    return sorted_path


def get_faces(edges):
    # go over all new edges
    # each can have max 2 faces
    # for each face do bfs and find shortest cycle

    edges_used = {}
    for edge in edges:
        edges_used[edge] = len(edge.link_faces)

    faces = []

    queue = []

    # initialize queue
    for edge in edges:
        path = [edge]
        depth = 0
        queue.append((edge.verts[1], path, depth))


    # BFS to find shortest cycle
    while len(queue) > 0:
        last_vertex, path, depth = queue.pop(0)
        
        if depth > 16:
            break

        elif last_vertex == path[0].verts[0]:
            # found face

            # check if edges are not used
            used = False
            for edge in path:
                if edges_used[edge] > 1:
                    used = True
                    break

            if not used:
                # check if face is not already in faces
                face = get_vertex_path(path)

                if face not in faces:
                    # add face
                    faces.append(get_vertex_path(path))
                    for edge in path:
                        edges_used[edge] += 1

        else:
            # get all edges that are connected to vert
            # check that edge is not used
            # edges_of_last_vertex = [edge for edge in last_vertex.link_edges if edge not in path and ((edge in edges_used and edges_used[edge] < 2) or (len(edge.link_faces) < 2))]
            edges_of_last_vertex = []
            for edge in last_vertex.link_edges:
                if edge not in path:
                    if edge in edges_used:
                        if edges_used[edge] < 2:
                            edges_of_last_vertex.append(edge)
                    elif len(edge.link_faces) < 2:
                        edges_of_last_vertex.append(edge)
                        edges_used[edge] = len(edge.link_faces)
            

            # add edges to queue
            for edge in edges_of_last_vertex:
                temp_path = path.copy()
                temp_path.append(edge)
                queue.append((edge.other_vert(last_vertex), temp_path, depth + 1))
                

    return faces
        


        



                






