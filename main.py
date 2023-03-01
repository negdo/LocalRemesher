import bpy
import bmesh
import numpy as np
import mathutils


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
    # print("")
    # print("edge: ", edge.index)
    selected_edges_left = selectNext(edge, edge.verts[0], [edge], illegal_edges)
    # print("left:", len(selected_edges_left))
    selected_edges_right = selectNext(edge, edge.verts[1], [edge], illegal_edges)
    # print("right:", len(selected_edges_right))

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

    # if lines are close to parralel return None
    if np.linalg.norm(n) < 0.2:
        return None
    
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


def select_convex(bm, verts, bm_original):
    # deselect all geometry
    for geom in bm.verts[:] + bm.edges[:] + bm.faces[:]:
        geom.select_set(False)
    for geom in bm_original.verts[:] + bm_original.edges[:] + bm_original.faces[:]:
        geom.select_set(False)

    #  generate convex hull of selected faces
    hull = bmesh.ops.convex_hull(bm, input=verts, use_existing_faces=True)
    overlaps_temp = hull["geom_holes"]
    overlaps = []
    print("overlaps:", len(overlaps_temp))
    for overlap in overlaps_temp:
        print(overlap)

    # delete all geometry that is not part of the convex hull
    not_selected = [vert for vert in bm.verts[:] if not vert.select]
    bmesh.ops.delete(bm, geom=not_selected, context="VERTS")
            
    # build BVH tree of convex hull
    bvh = mathutils.bvhtree.BVHTree.FromBMesh(bm)

    # select faces on original bmesh that are inside the convex hull
    # use raycasting in BVH tree to check normals
    for face in bm_original.faces:
        #check if face is overlapping
        for overlap in overlaps:
            if face.index == overlap:
                # face.select_set(True)
                pass


        # get center of face
        center = face.calc_center_median()
        # get normal of face
        normal = face.normal
        
        # raycast in BVH tree
        hit, hit_normal, face_index, distance = bvh.ray_cast(center, normal)
        if hit is None:
            hit, hit_normal, face_index, distance = bvh.ray_cast(center, -normal)


        # if normal is negative, face is inside the convex hull
        if hit is not None and hit_normal.dot(normal) > 0:
            face.select_set(True)

    
    



# set mode to object mode and init bmesh
bpy.ops.object.mode_set(mode='OBJECT')
me = bpy.context.object.data
bm = bmesh.new()
bm.from_mesh(me)

bm2 = bmesh.new()
bm2.from_mesh(me)

bm3 = bmesh.new()
bm3.from_mesh(me)



# get selected stuff
faces_selected = [face for face in bm.faces if face.select]
faces_selected2 = [face for face in bm2.faces if face.select]
verts_selected = [vert for face in faces_selected for vert in face.verts]
verts_selected2 = [vert for face in faces_selected2 for vert in face.verts]
# delete duplicate vertices
verts_selected = list(set(verts_selected))
verts_selected2 = list(set(verts_selected2))


select_convex(bm2, verts_selected2, bm3)


edges_of_faces = [edge for face in faces_selected for edge in face.edges]
# only vertices that are connected to at least one outside face are starting vertices
starting_verts = [vert for vert in verts_selected if len([face for face in vert.link_faces if not face.select]) > 0]
edges_of_verts = [edge for vert in starting_verts for edge in vert.link_edges]
# only edges that are connected to at least one outside face are starting edges
starting_edges = [edge for edge in edges_of_verts if len([face for face in edge.link_faces if not face.select]) > 0]
# illegal edges are inside the selected faces and do not connect to any outside face
illegal_edges = [edge for edge in edges_of_faces if len([face for face in edge.link_faces if not face.select]) == 0]
# semi illegal edges are on edge of selected faces
semi_illegal_edges = [edge for edge in edges_of_faces if len([face for face in edge.link_faces if face.select]) > 0]





# get loops and decide which edges to connect
connecting_vertices = []
used_edges = []

for edge in starting_edges:
    # check if edge is already used
    if edge not in used_edges:
        loop = get_loop(edge, illegal_edges, semi_illegal_edges)
        
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
                distances = np.zeros(4)
                distances[0] = np.linalg.norm(edge.verts[0].co - loop[-1].verts[0].co)
                distances[1] = np.linalg.norm(edge.verts[0].co - loop[-1].verts[1].co)
                distances[2] = np.linalg.norm(edge.verts[1].co - loop[-1].verts[0].co)
                distances[3] = np.linalg.norm(edge.verts[1].co - loop[-1].verts[1].co)

                # get index of the 2 closest vertices
                index = np.argmin(distances)
                if index == 0:
                    v1 = edge.verts[0]
                    v2 = loop[-1].verts[0]
                elif index == 1:
                    v1 = edge.verts[0]
                    v2 = loop[-1].verts[1]
                elif index == 2:
                    v1 = edge.verts[1]
                    v2 = loop[-1].verts[0]
                else:
                    v1 = edge.verts[1]
                    v2 = loop[-1].verts[1]


            # add edge to used edges
            used_edges.append(loop[0])
            used_edges.append(loop[-1])

            connecting_vertices.append([v1, v2])



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
            bm.edges.new([path[k], path[k+1]])
        except:
            # print("edge already exists")
            # print(path[k].index, path[k+1].index)
            pass


    
        
                                    
# write the bmesh back to the mesh
bm3.to_mesh(me)
bpy.ops.object.mode_set(mode='EDIT')