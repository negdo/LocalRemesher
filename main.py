import bpy
import bmesh
import numpy as np
import mathutils


def select_next(current_edge, vertex, selected_edges, illegal_edges):
    
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
            select_next(new_edges[0], new_edges[0].other_vert(vertex), selected_edges, illegal_edges)

    # if there are two edges, find the most parralel one
    elif len(new_edges) >= 2:
        # get angles between edges
        angles = [abs(np.dot((edge.verts[0].co - edge.verts[1].co)/np.linalg.norm(edge.verts[0].co - edge.verts[1].co), (current_edge.verts[0].co - current_edge.verts[1].co)/np.linalg.norm(current_edge.verts[0].co - current_edge.verts[1].co))) for edge in new_edges]

        index = np.argmax(angles)

        if new_edges[index] not in selected_edges and new_edges[index] not in illegal_edges:
            selected_edges.append(new_edges[index])

            # continue with the next vertex recursively
            select_next(new_edges[index], new_edges[index].other_vert(vertex), selected_edges, illegal_edges)

    # check, if any edges are close to parralel
    elif len(new_edges) == 0:
        # get angles between edges
        angles = [abs(np.dot((edge.verts[0].co - edge.verts[1].co)/np.linalg.norm(edge.verts[0].co - edge.verts[1].co), (current_edge.verts[0].co - current_edge.verts[1].co)/np.linalg.norm(current_edge.verts[0].co - current_edge.verts[1].co))) for edge in edges]

        index = np.argmax(angles)

        if edges[index] not in selected_edges and edges[index] not in illegal_edges:
            if angles[index] > 0.9:
                selected_edges.append(edges[index])

                # continue with the next vertex recursively
                select_next(edges[index], edges[index].other_vert(vertex), selected_edges, illegal_edges)

        

    return selected_edges


def get_loop(edge, illegal_edges):
    # go both directions
    selected_edges_left = select_next(edge, edge.verts[0], [edge], illegal_edges)
    selected_edges_right = select_next(edge, edge.verts[1], [edge], illegal_edges)

    if len(selected_edges_left) == 1:
        return selected_edges_right
    elif len(selected_edges_right) == 1:
        return selected_edges_left

    return [edge]


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
    # distance = np.dot(n, (r1 - r2)) / np.linalg.norm(n)

    # calculate t1 and t2
    t1 = np.dot(np.cross(e2, n), (r2 - r1)) / np.dot(n, n)
    t2 = np.dot(np.cross(e1, n), (r2 - r1)) / np.dot(n, n)

    # calculate points on lines
    p1 = r1 + t1 * e1
    p2 = r2 + t2 * e2

    # check if t1 and t2 are in range of line
    if t1 < 0 or t1 > 1 or t2 < 0 or t2 > 1:
        print("t1 or t2 out of range")
        return None

    # return middle point
    return (p1 + p2) / 2


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


def print_paths(intersected_verts):
    print("Intersected vertices:")
    for i in intersected_verts:
        print([v.index for v in i])


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


def select_faces_inside_convex_hull(bm, verts, bm_original):
    # deselect all geometry
    for geom in bm.verts[:] + bm.edges[:] + bm.faces[:]:
        geom.select_set(False)
    for geom in bm_original.verts[:] + bm_original.edges[:] + bm_original.faces[:]:
        geom.select_set(False)

    #  generate convex hull of selected faces
    hull = bmesh.ops.convex_hull(bm, input=verts, use_existing_faces=True)


    # delete all geometry that is not part of the convex hull
    not_selected = [vert for vert in bm.verts[:] if not vert.select]
    bmesh.ops.delete(bm, geom=not_selected, context="VERTS")
            
    # build BVH tree of convex hull
    bvh = mathutils.bvhtree.BVHTree.FromBMesh(bm)

    # select faces on original bmesh that are inside the convex hull
    # use raycasting in BVH tree to check normals
    for face in bm_original.faces:

        # get center of face
        center = face.calc_center_median()
        # get normal of face
        normal = face.normal
        
        # raycast in BVH tree
        hit, hit_normal, face_index, distance = bvh.ray_cast(center, normal)

        if hit is None:
            hit, hit_normal, face_index, distance = bvh.ray_cast(center, -normal)

        
        # if normal is negative, face is inside the convex hull
        if hit is not None and (hit_normal.dot(normal) > 0 or distance < 0.0001):
            face.select_set(True)
            # print("hit:", hit)
            # print("hit_normal:", hit_normal)
            # print("face_index:", face_index)
            # print("distance:", distance)
            # print("normal:", normal)
            # print(hit_normal.dot(normal))


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
    

def edge_similarity_weight(edge1, edge2, avg_dist, avg_direction):
    # get vectors of the two edges
    edge1_vec = (edge1.verts[0].co - edge1.verts[1].co)/np.linalg.norm(edge1.verts[0].co - edge1.verts[1].co)
    edge2_vec = (edge2.verts[0].co - edge2.verts[1].co)/np.linalg.norm(edge2.verts[0].co - edge2.verts[1].co)
    
    # get closest vertices
    vert1, vert2 = closest_vertices(edge1, edge2)

    # check if distance is too small
    dist = np.linalg.norm(np.array(vert1.co) - np.array(vert2.co))
    if dist < 0.0001:
        return 0, 0, 0

    # edge 3 - edge between vert1 and vert2
    edge3_vec = (vert1.co - vert2.co)/np.linalg.norm(vert1.co - vert2.co)

    # get angle offset of edges
    angle = np.abs(np.dot(edge1_vec, edge3_vec)) + np.abs(np.dot(edge2_vec, edge3_vec))

    # get distance between vertices
    dist = np.linalg.norm(np.array(vert1.co) - np.array(vert2.co))

    # get angle offset from average vector
    # normalise the resulting vector
    avg_angle_diff = np.abs(np.dot(avg_direction, edge3_vec))
    if avg_angle_diff < 0.5:
        # if less than 0.5, they are closer to perpendicular
        avg_angle_diff = 1 - avg_angle_diff
        

    if dist == 0 or angle < 0.7:
        return 0, 0, 0

    # get weight
    weight = angle * avg_angle_diff**2 / (dist + avg_dist)

    return weight, vert1, vert2
    


# set mode to object mode and init bmesh
bpy.ops.object.mode_set(mode='OBJECT')
me = bpy.context.object.data
bm = bmesh.new()
bm.from_mesh(me)

bm2 = bmesh.new()
bm2.from_mesh(me)

bm3 = bmesh.new()
bm3.from_mesh(me)



# get selected stuff for generating convex hull
faces_selected = [face for face in bm2.faces if face.select]
verts_selected = list(set(vert for face in faces_selected for vert in face.verts))

select_faces_inside_convex_hull(bm2, verts_selected, bm3)


# get list of selected faces - including the ones in the convex hull
faces_selected = [face for face in bm.faces if face.select]
verts_selected = list(set(vert for face in faces_selected for vert in face.verts))

edges_of_faces = [edge for face in faces_selected for edge in face.edges]
# only vertices that are connected to at least one outside face are starting vertices
starting_verts = [vert for vert in verts_selected if len([face for face in vert.link_faces if not face.select]) > 0]
print("starting_verts:", starting_verts)
edges_of_verts = [edge for vert in starting_verts for edge in vert.link_edges]
# only edges that are connected to at least one outside face are starting edges
starting_edges = [edge for edge in edges_of_verts if len([face for face in edge.link_faces if not face.select]) > 0]
# illegal edges are inside the selected faces and do not connect to any outside face
illegal_edges = [edge for edge in edges_of_faces if len([face for face in edge.link_faces if not face.select]) == 0]
# semi illegal edges are on edge of selected faces
semi_illegal_edges = [edge for edge in edges_of_faces if len([face for face in edge.link_faces if face.select]) > 0]
# average length of starting edges
avg_length = np.mean([edge.calc_length() for edge in starting_edges])
# average direction of starting edges
avg_direction = np.mean([(edge.verts[0].co - edge.verts[1].co)//np.linalg.norm(edge.verts[0].co - edge.verts[1].co) for edge in starting_edges], axis=0)




# get loops and decide which edges to connect
connecting_vertices = []
used_edges = []

for edge in starting_edges:
    # check if edge is already used
    if edge not in used_edges:
        loop = get_loop(edge, illegal_edges)
        
        # start and end of loop are in starting edges, we will probbably want to connect them later
        if len(loop) > 2 and loop[-1] in starting_edges and loop[-1] not in used_edges and loop[-1] != loop[0]:
            
            
            # choose vertex that is in starting_verts and append tuple of both vertices
            if edge.verts[0] in starting_verts and edge.verts[1] not in starting_verts:
                v1 = edge.verts[0]
            elif edge.verts[1] in starting_verts and edge.verts[0] not in starting_verts:
                v1 = edge.verts[1]
            else:
                v1 = None

            # the other end
            if loop[-1].verts[0] in starting_verts and loop[-1].verts[1] not in starting_verts:
                v2 = loop[-1].verts[0]
            elif loop[-1].verts[1] in starting_verts and loop[-1].verts[0] not in starting_verts:
                v2 = loop[-1].verts[1]
            else:
                v2 = None

            # if both vertices of edge are in starting verts, we have to choose the closer one
            if v1 is None and v2 is not None:
                if np.linalg.norm(v2.co - edge.verts[0].co) < np.linalg.norm(v2.co - edge.verts[1].co):
                    v1 = edge.verts[0]
                else:
                    v1 = edge.verts[1]
            elif v2 is None and v1 is not None:
                if np.linalg.norm(v1.co - loop[-1].verts[0].co) < np.linalg.norm(v1.co - loop[-1].verts[1].co):
                    v2 = loop[-1].verts[0]
                else:
                    v2 = loop[-1].verts[1]
            elif v1 is None and v2 is None:
                # find closest 2 vertices from each edge
                v1, v2 = closest_vertices(edge, loop[-1])


            # add edge to used edges
            used_edges.append(loop[0])
            used_edges.append(loop[-1])

            connecting_vertices.append([v1, v2])


# connecting the rest of vertices
# edges that are facing the same direction have higher priority
# vertices that are closer to each other have higher priority
# calculate weight for each combination of edges
# use priority queue to get the best combination

edges_to_connect = [edge for edge in edges_of_verts if edge not in used_edges]

print(len(edges_to_connect))

class potential_edge:
    def __init__(self, weight, v1, v2):
        self.weight = weight
        self.v1 = v1
        self.v2 = v2

    def __lt__(self, other):
        return self.weight < other.weight

# init priority queue
import heapq
pq = []
heapq.heapify(pq)

for i in range(len(edges_to_connect)-1):
    for j in range(i+1, len(edges_to_connect)):
        # get weight
        weight, v1, v2 = edge_similarity_weight(edges_to_connect[i], edges_to_connect[j], avg_length, avg_direction)

        # check if edge would make sense
        # both vertices have to be in starting_verts
        if v1 not in starting_verts or v2 not in starting_verts:
            continue

        # check if edge already exists
        if v2 in v1.link_edges:
            continue

        # check if weight is not 0
        if weight == 0:
            continue

        # add to priority queue
        heapq.heappush(pq, potential_edge(weight, v1, v2))

used_dict = {}

while len(pq) > 0:
    # get best combination
    edge_to_be = heapq.heappop(pq)
    

    # check if vertices are already used
    # if vert1 not in used_dict and vert2 not in used_dict:
    if edge_to_be.v1 not in used_dict and edge_to_be.v2 not in used_dict:
        print("adding edge")
        used_dict[edge_to_be.v1] = True
        used_dict[edge_to_be.v2] = True
        connecting_vertices.append([edge_to_be.v1, edge_to_be.v2])

    

    






# add intersecting points
for i in range(len(connecting_vertices)-1):
    for j in range(i+1, len(connecting_vertices)):
        # get intersection of two edges
        intersect = get_intersection(connecting_vertices[i][0].co, connecting_vertices[i][1].co, connecting_vertices[j][0].co, connecting_vertices[j][1].co)

        if intersect is not None:
            # project intersect on original geometry
            intersect = project_point_to_faces(intersect)

            # create new vertex
            new_vert = bm.verts.new(intersect)
            connecting_vertices[i].append(new_vert)
            connecting_vertices[j].append(new_vert)
    

# delete starting faces
bmesh.ops.delete(bm, geom=faces_selected, context="FACES")

# create new edges
for i in range(len(connecting_vertices)):
    # connect vertices in shortest path
    path = shortest_path(connecting_vertices[i])
    # print([v.index for v in path])
    for k in range(len(path)-1):
        try:
            edge = bm.edges.new([path[k], path[k+1]])
            edge.crease = 1
        except:
            # print("edge already exists")
            # print(path[k].index, path[k+1].index)
            pass


    
        
                                    
# write the bmesh back to the mesh
bm.to_mesh(me)
bpy.ops.object.mode_set(mode='EDIT')