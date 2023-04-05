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
    face_verts = list(face.verts)
    half1 = []
    half2 = []
    stage = 0

    for i in range(len(face_verts)):
        if stage == 0:
            if face_verts[i] == start:
                stage = 1
            else:
                half1.append(face_verts[i])
        elif stage == 1:
            if face_verts[i] == end:
                stage = 2
            else:
                half2.append(face_verts[i])
        elif stage == 2:
            half1.append(face_verts[i])


    convex_weight = 1
    if is_covex(half1, verts_bool=True):
        convex_weight *= 10
    if is_covex(half2, verts_bool=True):
        convex_weight *= 10

    return avg_angle_diff * convex_weight / dist
    #return avg_angle_diff / dist



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
    

    if len(edges["edges"]) == 0:
        return None, None
    edge = edges["edges"][0]
    update_elements(edge)

    # if number of original faces was 7 or more, add another vertex in the middle
    if full >= 7:
        bmesh.ops.subdivide_edges(bm, edges=[edge], cuts=1, use_grid_fill=True)
        update_elements(edges)

    if len(edge.link_faces) != 2:
        print("ERROR: edge does not have 2 faces")
        return None, None

    # return new faces
    return edge.link_faces[0], edge.link_faces[1]



def convert_5gon_to_6gon(bm, face, avg_direction):
    # split 5gon in half
    # split on edge in the middle and connect with opposite vertex
    max_weight = 0
    subdivide_edge = None

    for edge in face.edges:
        # find the middle point in current edge
        middle_point = (edge.verts[0].co + edge.verts[1].co) / 2

        update_elements(edge)

        # find the opposite vertex
        linked_edges_verts = set()
        for vert in edge.verts:
            for linked_edge in vert.link_edges:
                for vert in linked_edge.verts:
                    linked_edges_verts.add(vert)
        
        # find the opposite vertex
        opposite_vert = [vert for vert in face.verts if vert not in linked_edges_verts]
        if len(opposite_vert) != 1:
            print("ERROR: opposite vert not found")
            continue
        else:
            opposite_vert = opposite_vert[0]


        # get weight of this edge
        weight = get_edge_split_weight(face, middle_point, opposite_vert.co, avg_direction, direct_coords=True)

        if weight > max_weight:
            max_weight = weight
            subdivide_edge = edge

    
    # subdivide edge
    edges = bmesh.ops.subdivide_edges(bm, edges=[subdivide_edge], cuts=1, use_grid_fill=True)
    update_elements(edges)
    new_vert = edges["geom_split"][0]
    changed_faces = new_vert.link_faces

    # return faces neighboring new vert
    return [face for face in changed_faces if face.is_valid] + [face]



def subdivide_faces_to_quads(bm, faces, avg_direction):

    created_faces = set(faces)

    queue = []

    # add weighted faces to queue
    for face in faces:
        queue.append(Weighted_face(face))

    heapq.heapify(queue)

    while len(queue) > 0:
        weighted_face = heapq.heappop(queue)
        face = weighted_face.face
        if not face.is_valid:
            print("ERROR: invalid face")
            continue
        elif len(face.verts) == 4:
            # quads are great, no need to change them
            continue
        elif len(face.verts) == 5:
            # split one of the edges
            # connect new edge with opposite one
            faces_to_update = convert_5gon_to_6gon(bm, face, avg_direction)
            bm.faces.ensure_lookup_table()
            # add new split faces to queue
            for face in faces_to_update:
                if face is None:
                    print("ERROR: face is None")
                    continue
                heapq.heappush(queue, Weighted_face(face))
                created_faces.add(face)
        elif len(face.verts) > 5:
            # split face in half
            faces_to_update = split_face(bm, face, avg_direction)
            bm.faces.ensure_lookup_table()
            for face in faces_to_update:
                if face is None:
                    print("ERROR: face is None, >5")
                    continue
                heapq.heappush(queue, Weighted_face(face))
                created_faces.add(face)
        else:
            print("ERROR: face with less than 4 vertices")

    
    return [face for face in created_faces if face.is_valid]
       

def relax_vertices(all_verts, iterations=1, translation_factor=0.1):
    # get all iner verts that will be moved
    verts = []
    for vert in all_verts:
        out_verts_number = len([edge.other_vert(vert) for edge in vert.link_edges if edge.other_vert(vert) not in all_verts])
        if out_verts_number == 0:
            verts.append(vert)

    # make areas of faces more even
    edges = set(edge for vert in verts for edge in vert.link_edges)

    # do n iterations
    for iter in range(iterations):

        distances = [edge.calc_length() for edge in edges]
        global_avg_distance = sum(distances) / len(distances)

        for vert in verts:
            # for each vert get distances and vectors to all connected verts
            vectors = []
            skip = False

            # get vectors to all connected verts
            connected_verts = set(edge.other_vert(vert) for edge in vert.link_edges)
            for temp_vert in connected_verts:
                vectors.append(temp_vert.co - vert.co)


            # get vertices of connected faces
            verts_of_connected_faces = set(vert for face in vert.link_faces for vert in face.verts if vert not in connected_verts)

            # get vectors to all vertices of connected faces
            # scale them by 1/sqrt(2)
            for temp_vert in verts_of_connected_faces:
                vectors.append((temp_vert.co - vert.co) / math.sqrt(2))
            

            # get average vector distance
            avg_distance = np.average(np.array([vector.length for vector in vectors])) + global_avg_distance*0.01
            global_diffence = abs(global_avg_distance - avg_distance) + avg_distance*0.01

            # get translation vector
            translation_vector = mathutils.Vector((0, 0, 0))

            for vector in vectors:
                translation_vector += vector.normalized() * (vector.length - avg_distance) * translation_factor * global_diffence

            # count number of convex neighbor faces
            convex_faces = 0
            for face in vert.link_faces:
                if is_covex(face):
                    convex_faces += 1

            # translate vert
            vert.co += translation_vector

            # count number of convex neighbor faces after translation
            convex_faces_after = 0
            for face in vert.link_faces:
                if is_covex(face):
                    convex_faces_after += 1
            
            # if number of convex faces is lower, revert translation
            if convex_faces_after < convex_faces:
                vert.co -= translation_vector


def update_elements(elements):
    try:
        elements.index_update()
    except:
        pass
    try:
        elements.ensure_lookup_table()
    except:
        pass



def improve_edge_flow_direction(bm, all_verts, avg_direction):
    # find verts with only two edges, they maybe should be split differently

    # get all iner verts that will be checked
    verts = []
    for vert in all_verts:
        out_verts_number = len([edge.other_vert(vert) for edge in vert.link_edges if edge.other_vert(vert) not in all_verts])
        if out_verts_number == 0:
            update_elements(vert)
            verts.append(vert)

    
    for vert in verts:
        if not vert.is_valid:
            print("ERROR: vert not valid")
            continue
        # check if vert has only two edges
        if len(vert.link_edges) == 2:
            # get neighbor faces
            update_elements(vert.link_edges)

            linked_faces = [face for face in vert.link_faces if face.is_valid]
            update_elements(linked_faces)
            update_elements(vert)
            linked_faces = [face for face in linked_faces if face.is_valid]

            # check that linked faces is list and has two elements
            if len(linked_faces) != 2:
                print("ERROR: not two faces linked to vert or not list")
                continue

            # check that all faces are quads or triangles
            if abs(len(linked_faces[0].verts)-3.5) > 0.5 or abs(len(linked_faces[1].verts)-3.5) > 0.5:
                print("ERROR: not quad or triangle")
                continue

            # dissolve vert
            bmesh.ops.dissolve_verts(bm, verts=[vert])
            update_elements(vert)
            update_elements(linked_faces)

            if not linked_faces[0].is_valid or not linked_faces[1].is_valid:
                print("ERROR: face not valid")
                continue

            # get edge to dissolve - intersection of face.edges
            edges_dissolve = [edge for edge in linked_faces[0].edges if edge in linked_faces[1].edges]

            if len(edges_dissolve) != 1:
                print("ERROR: edge not found")
                continue

            # dissolve edge
            bmesh.ops.dissolve_edges(bm, edges=edges_dissolve)
            update_elements(edges_dissolve)


        elif len(vert.link_edges) == 3:
            # get all three neighbor faces
            linked_faces = [face for face in vert.link_faces]

            # check that linked faces is list and has three elements
            if len(linked_faces) != 3:
                continue

            # check that all faces are quads
            if len(linked_faces[0].verts) != 4 or len(linked_faces[1].verts) != 4 or len(linked_faces[2].verts) != 4:
                continue

            # get set of verts around the faces
            verts1 = set(linked_faces[0].verts)
            verts2 = set(linked_faces[1].verts)
            verts3 = set(linked_faces[2].verts)

            inside_verts = verts1.intersection(verts2).intersection(verts3)
            around_verts = verts1.union(verts2).union(verts3).difference(inside_verts)
            around_verts = list(around_verts)

            if len(around_verts) != 6:
                print("ERROR: not 6 verts around faces")
                continue

            # dissolve vert
            bmesh.ops.dissolve_verts(bm, verts=[vert])
            update_elements(vert)

            # get face that is linked to all 6 verts
            linked_faces = set(around_verts[0].link_faces)
            for i in range(1, 6):
                linked_faces = linked_faces.intersection(around_verts[i].link_faces)
            
            if len(linked_faces) != 1:
                print("ERROR: not one face linked to all verts")
                continue

            around_verts = linked_faces.pop().verts
            
            # opposite vertices will be connected
            connections = [(around_verts[0], around_verts[3]), (around_verts[1], around_verts[4]), (around_verts[2], around_verts[5])]

            best_connection = None
            max_weight = 0

            for connection in connections:
                weight = get_edge_split_weight(bm, connection[0], connection[1], avg_direction)
                if weight > max_weight:
                    max_weight = weight
                    best_connection = connection

            if best_connection is None:
                print("ERROR: best connection not found")
                continue

            # split edge
            bmesh.ops.connect_vert_pair(bm, verts=best_connection)
            update_elements(best_connection)
            update_elements(bm.edges)
            update_elements(bm.verts)
            update_elements(bm.faces)



def smooth_edge_flow(bm, all_verts):
    pass            

        


                


    











    
                
            








