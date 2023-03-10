import bpy
import bmesh
import numpy as np
import mathutils
from scipy.optimize import linear_sum_assignment
import sys
import heapq

sys.path.append("C:/Users/miham/Documents/3d/blender/scripts/local remesh/LocalRemesher/rewrite")
from selection_utils import *
from other_utils import *
from Directed_edge import *
from loop_utils import *
from building_utils import *
from improving_utils import *

############## INIT #################################

# set mode to object mode
bpy.ops.object.mode_set(mode='OBJECT')

# init bmesh objects
me = bpy.context.object.data
bm = bmesh.new()
bm.from_mesh(me)
bm_for_convex = bmesh.new()
bm_for_convex.from_mesh(me)


############## SELECTION ############################

# get selected faces and their vertices for convex mesh
faces_selected, verts_selected = get_selected(bm_for_convex)

# make the selection convex
select_faces_inside_convex_hull(bm_for_convex, verts_selected, bm)

# get selected faces and their vertices
faces_selected, verts_selected = get_selected(bm)

# get starting edges, illegal edges and average length of starting edges
# starting edges are vertices with direction, where newly created edges will start
starting_edges, illegal_edges, avg_length = get_starting(faces_selected, verts_selected)


############## LOOPS SEARCH ############################

# pairs of vertices that will be connected to new edges
connecting_vertices = []
used_edges = []

# for each edge look for loop ending in another starting edge
for edge in starting_edges:
    if edge not in used_edges:
        end = get_loop(edge.edge, illegal_edges)
        
        # check if new edge would be suitable
        if end is not None and edge.edge != end.edge and edge.vert != end.vert:
            # check if end is not used and is included in starting edges
            if in_list_directed_edge(end, starting_edges) and not in_list_directed_edge(edge, used_edges):
                
                # check if edge already exists
                if end.vert not in [e.other_vert(edge.vert) for e in edge.vert.link_edges]:

                    # mark these edges as used and 
                    used_edges.append(edge)
                    used_edges.append(end)
                    connecting_vertices.append([edge.vert, end.vert])



############## DEFINE EDGE FLOW DIRECTION ##############

# TODO: make better algorithm for defining edge flow direction
# could use best weighted loop, comparing angle of continuation edges
if len(connecting_vertices) > 0:
    # use average direction of already defined loops
    avg_direction = connecting_vertices[0][1].co - connecting_vertices[0][0].co
    avg_direction.normalized()
else:
    # use average direction of starting edges
    avg_direction = starting_edges[0].edge.verts[0].co - starting_edges[0].edge.verts[1].co


# delete old faces
bmesh.ops.delete(bm, geom=faces_selected, context="FACES")


############## CONNECTING OTHER VERTICES ###############

# edges that are facing the avg_direction (or perpendicular) have higher priority
# calculate weight for each combination of edges

edges_to_connect = [edge for edge in starting_edges if edge not in used_edges]

# init priority queue
pq = []
heapq.heapify(pq)

for i in range(len(edges_to_connect)-1):
    for j in range(i+1, len(edges_to_connect)):
        # check edge already exists in original mesh
        if edges_to_connect[j].vert not in [e.other_vert(edges_to_connect[i].vert) for e in edges_to_connect[i].vert.link_edges]:
            # add weighted edge to priority queue
            heapq.heappush(pq, Weighted_edge(edges_to_connect[i], edges_to_connect[j], avg_direction))

used_edges = set()

# get edges with highest weight from priority queue
while len(pq) > 0:
    weighted_edge = heapq.heappop(pq)

    # check if weight is 0
    if weighted_edge.weight == 0:
        continue

    # check if start or end is already used
    if weighted_edge.start in used_edges or weighted_edge.end in used_edges:
        continue

    used_edges.add(weighted_edge.start)
    used_edges.add(weighted_edge.end)
    connecting_vertices.append([weighted_edge.start.vert, weighted_edge.end.vert])





############## ITERSECTING POINTS #####################

# check intersection between each pair of lines
for i in range(len(connecting_vertices)-1):
    for j in range(i+1, len(connecting_vertices)):
        # get intersection point
        intersect = get_intersection(connecting_vertices[i][0].co, connecting_vertices[i][1].co, connecting_vertices[j][0].co, connecting_vertices[j][1].co)

        if intersect is not None:
            # project the point on original mesh
            intersect = project_point_to_faces(intersect)

            # create new vertex
            new_vert = bm.verts.new(intersect)
            connecting_vertices[i].append(new_vert)
            connecting_vertices[j].append(new_vert)


############## BUILDING MESH #########################

avg_new_edge_length = 0
vertices = set()
edges = []
outside_vertices = set()

# create new edges
for i in range(len(connecting_vertices)):
    # get shortest path including all vertices in line
    path = shortest_path(connecting_vertices[i])

    for j in range(len(path)-1):
        edge = bm.edges.new((path[j], path[j+1]))
        avg_new_edge_length += (path[j].co - path[j+1].co).length
        vertices.add(path[j])
        edges.append(edge)
    vertices.add(path[-1])
    outside_vertices.add(path[-1])
    outside_vertices.add(path[0])
    

vertices = list(vertices)
avg_new_edge_length /= len(connecting_vertices)


# merge vertices by distance
bmesh.ops.remove_doubles(bm, verts=vertices, dist=avg_new_edge_length/8)

# remove vertices that were merged from vertices list
vertices = [v for v in vertices if v.is_valid and v not in outside_vertices]
edges = [e for e in edges if e.is_valid]

edges = list(set(edge for vert in vertices for edge in vert.link_edges))


############## IPROVE MESH ###########################

# update indices of vertices

# find all triangles
triangles = get_triangles(edges)

triangles_faces = []
# fill triangles
for triangle in triangles:
    triangles_faces.append(bm.faces.new(triangle))

# dissolve triangles, calculate weight of each edge in triangle
pq = []
heapq.heapify(pq)

for triangle in triangles_faces:
    heapq.heappush(pq, Weighted_triangle(triangle, avg_direction))

# dissolve triangles with highest weight
while len(pq) > 0:
    weighted_triangle = heapq.heappop(pq)
    # check if face is valid / still exists
    if not weighted_triangle.triangle.is_valid:
        continue
    # dissolve triangle
    bmesh.ops.delete(bm, geom=[weighted_triangle.max_edge], context="EDGES")






############## END #####################################

sys.path.remove("C:/Users/miham/Documents/3d/blender/scripts/local remesh/LocalRemesher/rewrite")
del sys.modules["selection_utils"]
del sys.modules["other_utils"]
del sys.modules["Directed_edge"]
del sys.modules["loop_utils"]
del sys.modules["building_utils"]

# update bmesh
bm.to_mesh(me)
bm.free()
bm_for_convex.free()
bpy.ops.object.mode_set(mode='EDIT')




