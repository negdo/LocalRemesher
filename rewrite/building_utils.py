import bpy
import bmesh
import numpy as np
import mathutils
from scipy.optimize import linear_sum_assignment

# returns intersection of two lines in 3D
def get_intersection(r1, q1, r2, q2):
    # find a point that is closest intersection of two lines in 3D
    # https://math.stackexchange.com/questions/2213165/find-shortest-distance-between-lines-in-3d
    # r1 and q2 are points on the first line
    # r2 and q2 are points on the second line
    # find distance between the lines
    r1 = np.array(r1)
    q1 = np.array(q1)
    r2 = np.array(r2)
    q2 = np.array(q2)

    # get vectors of lines
    e1 = q1 - r1
    e2 = q2 - r2

    # get normal vector of plane defined by lines
    n = np.cross(e1, e2)
    
    # get distance between lines
    distance = np.dot(n, (r1 - r2)) / np.linalg.norm(n)

    # if distance is more than minimum half of length of the lines, return None
    distance_ratio = 2*distance / min(np.linalg.norm(e1), np.linalg.norm(e2))

    # calculate t1 and t2
    t1 = np.dot(np.cross(e2, n), (r2 - r1)) / np.dot(n, n)
    t2 = np.dot(np.cross(e1, n), (r2 - r1)) / np.dot(n, n)

    # calculate points on lines
    p1 = r1 + t1 * e1
    p2 = r2 + t2 * e2

    # check if t1 and t2 are in range of line
    if t1 < -0.05 or t1 > 1.05 or t2 < -0.05 or t2 > 1.05:
        return None
    
    # distance can be higher in the middle of the line, linearly changing function
    if 1 - 2 * abs(t1 - 0.5) * distance_ratio < 0.05 or 1 - 2 * abs(t2 - 0.5) * distance_ratio < 0.05:
        return None
    
    ratio = t1 / (t1 + t2)

    # return point of intersection
    return ratio * p1 + (1 - ratio) * p2


# returns sorted list of vertices to get shortest path
def shortest_path(vertices):
    # vertices - list of vertices
    # get the order in which they should be connected to form a shortest path

    path = [vertices[0]]
    end = vertices[1]
    vertices.pop(0)
    vertices.pop(0)
    
    while len(vertices) > 0:
        # get distance to all other vertices
        distances = [np.linalg.norm(np.array(vertex.co) - np.array(path[-1].co)) for vertex in vertices]
        # get index of the vertex with the shortest distance
        index = np.argmin(distances)
        # add vertex to path
        path.append(vertices[index])
        # remove vertex from list
        vertices.pop(index)

    path.append(end)

    return path


def preprate_bm_for_projection(bm, subdiv=0, delete_faces=True):
    # delete all faces that are not selected from bmesh
    if delete_faces:
        faces = [face for face in bm.faces if not face.select]
        bmesh.ops.delete(bm, geom=faces, context="FACES")

    # triangulate faces
    bmesh.ops.triangulate(bm, faces=bm.faces, quad_method="SHORT_EDGE")

    # subdivide faces
    if subdiv > 0:
        bmesh.ops.subdivide_edges(bm, edges=bm.edges, cuts=subdiv, use_grid_fill=True, smooth=1)
        
    bmesh.ops.triangulate(bm, faces=bm.faces, quad_method="SHORT_EDGE")

    # ensure local table is up to date
    bm.verts.ensure_lookup_table()
    bm.faces.ensure_lookup_table()

    vertices = [v.co for v in bm.verts]
    polygons = [[v.index for v in f.verts] for f in bm.faces]

    # build bvh tree
    bvh_tree = mathutils.bvhtree.BVHTree.FromPolygons(vertices, polygons, all_triangles=True)

    return bvh_tree

# projects point to original mesh, closest face
def project_point_to_faces(bvh_tree, point, bm):
    point = mathutils.Vector(point)

    # find closest face
    closest_face = bvh_tree.find_nearest(point)[2]
    if closest_face is None:
        return None
    closest_face = bm.faces[closest_face].verts

    # get closest point on face
    closest_point = mathutils.geometry.closest_point_on_tri(point, closest_face[0].co, closest_face[1].co, closest_face[2].co)

    return closest_point


def sort_vertices(vertex_path):
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

def get_vertex_path(edge_path):
    # get vertex path from edge path
    vertex_path = [edge_path[0].verts[0]]
    for edge in edge_path:
        vertex_path.append(edge.verts[0])
        vertex_path.append(edge.verts[1])
        # ensure indices are updated

    vertex_path = list(set(vertex_path))

    # sort path by index, but vertices have to be connected
    vertex_path.sort(key=lambda vert: vert.index)
    sorted_path = [vertex_path.pop(0)]
    for i in range(len(vertex_path)):
        linked_verts = [edge.other_vert(sorted_path[-1]) for edge in sorted_path[-1].link_edges if edge in edge_path]
        for j in range(len(vertex_path)):
            if vertex_path[j] in linked_verts:
                sorted_path.append(vertex_path.pop(j))
                break

        
    return sorted_path

    
def get_faces(edges, bm):
    # go over all new edges
    # each can have max 2 faces
    # for each face do bfs and find shortest cycle

    faces = []
    queue = []
    created_faces = []

    # initialize queue
    for edge in edges:
        if len(edge.link_faces) > 1:
            continue
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
                if len(edge.link_faces) > 1:
                    used = True
                    break
                
            if not used:
                # get ordered list of vertices
                face = get_vertex_path(path)

                if face not in faces:
                    # try to create face

                    try:
                        new_face = bm.faces.new(face)
                        created_faces.append(new_face)

                    except Exception as e:
                        print("ERROR: could not create face", str([vert.index for vert in face]))
                        print(e)
                        print("path", str([edge.index for edge in path]))
                        print("")

        else:
            # get all edges that are connected to vert
            # check that edge is not used
            edges_of_last_vertex = []
            for edge in last_vertex.link_edges:
                if edge.is_valid and edge not in path:
                    if len(edge.link_faces) < 2:
                        edges_of_last_vertex.append(edge)
            
            # add edges to queue
            for edge in edges_of_last_vertex:
                temp_path = path.copy()
                temp_path.append(edge)
                queue.append((edge.other_vert(last_vertex), temp_path, depth + 1))
                
    return created_faces





        





    

    



