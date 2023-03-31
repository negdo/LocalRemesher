import bpy
import bmesh
import numpy as np
import heapq

from other_utils import *
from Directed_edge import *


def get_edge_split_weight(face, start, end, avg_direction, direct_coords=False):
    # start and end are vertices
    # higher weight is better

    if direct_coords:
        vec = start - end
    else:
        vec = start.co - end.co
    # shorter distance is better
    dist = math.sqrt(vec.length)
    vec = vec.normalized()

    # compare to average direction
    avg_angle_diff = np.abs(np.dot(avg_direction, vec))
    if avg_angle_diff < 0.5:
        # if less than 0.5, they are closer to perpendicular
        avg_angle_diff = 1 - avg_angle_diff
    avg_angle_diff = (avg_angle_diff - 0.5) * 5

    # check if halffaces are convex
    # TODO
 

    return avg_angle_diff / dist

    




def split_face(bm, face, avg_direction):
    # split face in half with opposite vertices
    half = len(face.verts) // 2
    full = len(face.verts)

    # get all possible weighted edges
    splitting_edge = None
    max_weight = 0

    # find edge with highest weight
    if len(face.verts) % 2 == 0:
        # even number of vertices means only opposite vertices can be used
        for i in range(len(face.verts)-half):
            weight = get_edge_split_weight(face, face.verts[i], face.verts[(i+half) % full], avg_direction)
            if weight > max_weight:
                max_weight = weight
                splitting_edge = (face.verts[i], face.verts[(i+half) % full])

    else:
        # odd number means opposite is not strict
        for i in range(len(face.verts)-half):
            weight = get_edge_split_weight(face, face.verts[i], face.verts[(i+half) % full], avg_direction)
            if weight > max_weight:
                max_weight = weight
                splitting_edge = (face.verts[i], face.verts[(i+half) % full])
            
            weight = get_edge_split_weight(face, face.verts[i], face.verts[(i+half+1) % full], avg_direction)
            if weight > max_weight:
                max_weight = weight
                splitting_edge = (face.verts[i], face.verts[(i+half+1) % full])

    # create new edge and split face
    edges = bmesh.ops.connect_vert_pair(bm, verts=splitting_edge)
    edge = edges["edges"][0]
    print("new edge: ", edge)

    # return new faces
    return edge.link_faces[0], edge.link_faces[1]



def split_5gon(bm, face, avg_direction):
    # split 5gon in half
    # split on edge in the middle and connect with opposite vertex
    max_weight = 0
    subdivide_edge = None
    opposite_vert = None

    for edge in face.edges:
        # find the middle point in current edge
        middle_point = (edge.verts[0].co + edge.verts[1].co) / 2

        # find the opposite vertex
        linked_edges_verts = set()
        for vert in edge.verts:
            for linked_edge in vert.link_edges:
                for vert in linked_edge.verts:
                    linked_edges_verts.add(vert)
        
        # find the opposite vertex
        opposite_vert = [vert for vert in face.verts if vert not in linked_edges_verts][0]


        # get weight of this edge
        weight = get_edge_split_weight(face, middle_point, opposite_vert.co, avg_direction, direct_coords=True)

        if weight > max_weight:
            max_weight = weight
            subdivide_edge = edge
            opposite_vert = opposite_vert


    print("opposite_vert: ", opposite_vert)
    print("valid: ", opposite_vert.is_valid)
    
    # subdivide edge
    edges = bmesh.ops.subdivide_edges(bm, edges=[subdivide_edge], cuts=1, use_grid_fill=True)
    new_vert = edges["geom_split"][0]
    changed_faces = new_vert.link_faces

    print("new_vert: ", new_vert)
    print("valid: ", new_vert.is_valid)
    print("opposite_vert: ", opposite_vert)
    print("valid: ", opposite_vert.is_valid)
    

    

    # connect new vert with opposite one
    edges = bmesh.ops.connect_vert_pair(bm, verts=[new_vert, opposite_vert])
    edge = edges["edges"][0]

    # return new faces
    return [face for face in changed_faces if face.is_valid]



    



def subdivide_faces_to_quads(bm, faces, avg_direction):
    print("subdivide_faces_to_quads")
    print("faces: ", faces)

    queue = []

    # add weighted faces to queue
    for face in faces:
        queue.append(Weighted_face(face))

    heapq.heapify(queue)

    while len(queue) > 0:
        weighted_face = heapq.heappop(queue)
        print("weighted_face: ", weighted_face.face.index, len(weighted_face.face.verts))
        face = weighted_face.face

        if len(face.verts) == 4:
            continue
        elif len(face.verts) == 5:
            # split one of the edges
            # connect new edge with opposite one
            faces_to_update = split_5gon(bm, face, avg_direction)
            # add new split faces to queue
            for face in faces_to_update:
                heapq.heappush(queue, Weighted_face(face))
        elif len(face.verts) > 5:
            # split face in half
            print("splitting face")
            faces_to_update = split_face(bm, face, avg_direction)
            for face in faces_to_update:
                heapq.heappush(queue, Weighted_face(face))
        else:
            print("ERROR: face with less than 4 vertices")

        





    
                
            








