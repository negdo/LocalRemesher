import bpy
import bmesh
import numpy as np
import mathutils

# selects all vertices that are inside the convex hull of selected faces
def select_faces_inside_convex_hull(bm, verts, bm_original):
    # deselect all original geometry
    for geom in bm_original.verts[:] + bm_original.edges[:] + bm_original.faces[:]:
        geom.select_set(False)

    # delete all geometry that is not part of the convex hull
    to_delete = []
    for vert in bm.verts:
        if not vert.select:
            to_delete.append(vert)

    bmesh.ops.delete(bm, geom=to_delete, context="VERTS")

    #  generate convex hull of selected faces
    bmesh.ops.convex_hull(bm, input=verts)

    # build BVH tree of convex hull
    bvh = mathutils.bvhtree.BVHTree.FromBMesh(bm)


    # select vertices on original bmesh that are inside the convex hull
    # use raycasting in BVH tree to check normals
    for vert in bm_original.verts:

        # get center of face
        center = vert.co
        # get normal of face
        normal = mathutils.Vector((0, -1, 0))

        # raycast in BVH tree
        hit, hit_normal, face_index, distance = bvh.ray_cast(center, normal)

        if hit is None:
            hit, hit_normal, face_index, distance = bvh.ray_cast(center, -normal)

        if hit is not None:
            # if distance is small, select vertex
            if distance < 0.0001:
                vert.select = True

            # if we hit the backface, select vertex
            elif hit_normal.dot(normal) >= 0:
                vert.select = True


    bm_original.select_flush(True)


# from two edges returns closest vertex for each edge
def closest_vertices(edge1, edge2):
    # get vertices of the two edges
    vert1 = edge1.verts[0]
    vert2 = edge1.verts[1]
    vert3 = edge2.verts[0]
    vert4 = edge2.verts[1]

    # get distance between vertices
    dist1 = np.linalg.norm(np.array(vert1.co) - np.array(vert3.co))
    dist2 = np.linalg.norm(np.array(vert1.co) - np.array(vert4.co))
    dist3 = np.linalg.norm(np.array(vert2.co) - np.array(vert3.co))
    dist4 = np.linalg.norm(np.array(vert2.co) - np.array(vert4.co))

    # get index of the two vertices that are closest to each other
    index1 = np.argmin([dist1, dist2, dist3, dist4])

    # return vertices
    if index1 == 0:
        return vert1, vert3
    elif index1 == 1:
        return vert1, vert4
    elif index1 == 2:
        return vert2, vert3
    elif index1 == 3:
        return vert2, vert4


# calculates weight of how good would be edge between two vertices
def edge_similarity_weight(edge1, edge2, avg_direction):
    # higher weight is better

    # get vectors of the two edges
    edge1_vec = (edge1.edge.verts[0].co - edge1.edge.verts[1].co).normalized()
    edge2_vec = (edge2.edge.verts[0].co - edge2.edge.verts[1].co).normalized()

    # edge3 is the edge between the two vertices
    edge3_vec = edge1.vert.co - edge2.vert.co
    dist = np.linalg.norm(edge3_vec)
    edge3_vec = edge3_vec.normalized()

    # if distance is small, return 0
    if dist < 0.0001:
        return 0

    # get angle offset of edges - higher is more parallel
    angle = np.abs(np.dot(edge1_vec, edge3_vec)) + np.abs(np.dot(edge2_vec, edge3_vec))

    # get angle offset from average vector
    avg_angle_diff = np.abs(np.dot(avg_direction, edge3_vec))
    if avg_angle_diff < 0.5:
        # if less than 0.5, they are closer to perpendicular
        avg_angle_diff = 1 - avg_angle_diff

    # get weight
    weight = angle * avg_angle_diff**2

    return weight


# check if provided directed edge is in the list
def in_list_directed_edge(directed_edge, directed_edge_list):
    for directed_edge2 in directed_edge_list:
        if directed_edge.edge == directed_edge2.edge and directed_edge.vert == directed_edge2.vert:
            return True

    return False


def is_covex(face, verts_bool=False):
    # check if face is convex
    sign = 0
    if verts_bool:
        vertices = face
    else:
        vertices = face.verts
    
    # check all angles, if any is greater than 180, return False
    for i in range(len(vertices)):
    
        # get vectors of the two edges
        edge1_vec = (vertices[i].co - vertices[(i+1) % len(vertices)].co).normalized()
        edge2_vec = (vertices[(i+2) % len(vertices)].co - vertices[(i+1) % len(vertices)].co).normalized()

        if sign == 0:
            sign = np.sign(np.cross(edge1_vec, edge2_vec)[2])
        else:
            if sign != np.sign(np.cross(edge1_vec, edge2_vec)[2]):
                return False
        
    return True



def define_average_direction(edges):
    # make matrix of cosine distances between edges
    matrix = np.zeros((len(edges), len(edges)))
    for i in range(len(edges)):
        for j in range(len(edges)):
            edge1_vec = (edges[i].edge.verts[0].co - edges[i].edge.verts[1].co).normalized()
            edge2_vec = (edges[j].edge.verts[0].co - edges[j].edge.verts[1].co).normalized()
            matrix[i, j] = np.abs(np.dot(edge1_vec, edge2_vec))
            # adjusted cosine distance, that equaly weights 0, 90 and 180 degrees
            if matrix[i, j] < 0.5:
                matrix[i, j] = 1 - matrix[i, j]
    
    # find the highest sum of cosine distances of one edge
    max_sum = 0
    max_index = 0
    for i in range(len(edges)):
        sum = np.sum(matrix[i, :])
        if sum > max_sum:
            max_sum = sum
            max_index = i
    
    # return average direction - vector of edge on index max_index
    return (edges[max_index].edge.verts[0].co - edges[max_index].edge.verts[1].co).normalized()