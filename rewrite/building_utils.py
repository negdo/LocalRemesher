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
    if distance_ratio > 0.5:
        return None

    # calculate t1 and t2
    t1 = np.dot(np.cross(e2, n), (r2 - r1)) / np.dot(n, n)
    t2 = np.dot(np.cross(e1, n), (r2 - r1)) / np.dot(n, n)

    # calculate points on lines
    p1 = r1 + t1 * e1
    p2 = r2 + t2 * e2

    # check if t1 and t2 are in range of line
    if t1 < 0.001 or t1 > 0.999 or t2 < 0.01 or t2 > 0.999:
        return None
    
    # distance can be higher in the middle of the line, linearly changing function
    if 1 - 2 * abs(t1 - 0.5) * distance_ratio < 0 or 1 - 2 * abs(t2 - 0.5) * distance_ratio < 0:
        return None
    
    # return middle point
    return (p1 + p2) / 2


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


# projects point to original mesh, closest face
def project_point_to_faces(point):
    point = mathutils.Vector(point)
    # init new bmesh
    me = bpy.context.object.data
    bm = bmesh.new()
    bm.from_mesh(me)

    # delete all faces that are not selected from bmesh
    faces = [face for face in bm.faces if not face.select]
    bmesh.ops.delete(bm, geom=faces, context="FACES")

    # triangulate faces
    triangulated = bmesh.ops.triangulate(bm, faces=bm.faces, quad_method="SHORT_EDGE")
    tris = triangulated["faces"]

    # find closest face
    distances = [np.linalg.norm(face.calc_center_median() - point) for face in tris]
    index = np.argmin(distances)
    closest_face = tris[index]

    # get closest point on face
    closest_point = mathutils.geometry.closest_point_on_tri(point, closest_face.verts[0].co, closest_face.verts[1].co, closest_face.verts[2].co)

    return closest_point


# merges vertices that are closer than distance
def merge_by_distance(vertices, vertices_outside, distance, bm):
    # verticecs - list of all vertices
    # vertices_outside - set of vertices that are outside of the remesh area and should not be moved

    # distance matrix
    matrix = -np.ones((len(vertices), len(vertices)))
    for i in range(len(vertices)-1):
        for j in range(i+1, len(vertices)):
            matrix[i][j] = np.linalg.norm(np.array(vertices[i].co) - np.array(vertices[j].co))

    # get indices of vertices that are closer than distance
    coords = np.argwhere(matrix < distance)

    moved_vertices = set()

    # not perfect, if there are more than 5 vertices, they could potentialy not be merged, if the order is not right
    for x, y in coords:
        if matrix[x][y] == -1:
            continue
        
        outside1 = vertices[x] in vertices_outside or vertices[x] in vertices_outside
        outside2 = vertices[y] in vertices_outside or vertices[y] in vertices_outside
        # if both vertices are outside of the remesh area, skip
        if outside1 and outside2:
            continue
        # if one of the vertices is outside, move the other to it
        elif outside1:
            vertices[y].co = vertices[x].co
            moved_vertices.add(vertices[y])
        elif outside2:
            vertices[x].co = vertices[y].co
            moved_vertices.add(vertices[x])
        # if both vertices are inside, move them to the middle
        else:
            vertices[y].co = vertices[x].co
            moved_vertices.add(vertices[x])
            moved_vertices.add(vertices[y])
            bmesh.ops.remove_doubles(bm, verts=[vertices[x], vertices[y]], dist=0.001)

    
    #bmesh.ops.remove_doubles(bm, verts=vertices, dist=0.001)
    






        





    

    



