import bpy
import bmesh
import numpy as np
import mathutils
from scipy.optimize import linear_sum_assignment
import sys
import heapq
import os
import pathlib

# get file location
syspath = os.path.abspath(__file__)
syspath = pathlib.Path(syspath).parent.absolute()
syspath = pathlib.Path(syspath).parent.absolute()
syspath = str(syspath) + "\\rewrite"
sys.path.append(syspath)

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
bm_proj2 = bmesh.new()
bm_proj2.from_mesh(me)
bm_triangulated = bmesh.new()
bm_triangulated.from_mesh(me)


############## SELECTION ############################

# get selected faces and their vertices for convex mesh
faces_selected, verts_selected = get_selected(bm_for_convex)

# make the selection convex
#select_faces_inside_convex_hull(bm_for_convex, verts_selected, bm)

# get selected faces and their vertices
faces_selected, verts_selected = get_selected(bm)

# get starting edges, illegal edges and average length of starting edges
# starting edges are vertices with direction, where newly created edges will start
starting_edges, illegal_edges, avg_length = get_starting(faces_selected, verts_selected)


# pairs of vertices that will be connected to new edges
connecting_vertices = []
used_edges = []


############## SAMPLE PARTICLES ########################

# delete faces that are not selected
bmesh.ops.delete(bm_triangulated, geom=[f for f in bm_triangulated.faces if f.select == False], context="FACES")
# triangulate mesh
bmesh.ops.triangulate(bm_triangulated, faces=bm_triangulated.faces)
# calculate weights for each face - area of face
weights = [f.calc_area() for f in bm_triangulated.faces]
# define number of particles
num_particles = len(bm_triangulated.faces) * 10

# sample particles
particles_per_face = np.random.multinomial(num_particles, weights/np.sum(weights))
particles = []

for i in range(len(particles_per_face)):
    for j in range(particles_per_face[i]):
        # sample random point on triangle
        # vectors of triangle edges
        vec1 = bm_triangulated.faces[i].verts[0].co - bm_triangulated.faces[i].verts[1].co
        vec2 = bm_triangulated.faces[i].verts[0].co - bm_triangulated.faces[i].verts[2].co

        # random variables
        t1 = np.random.uniform(0, 1)
        t2 = np.random.uniform(0, 1)

        # if the sum is more than 1, put it back in the triangle
        if t1 + t2 > 1:
            t1 = 1 - t1
            t2 = 1 - t2
        
        # calculate point
        particles.append(bm_triangulated.faces[i].verts[0].co + t1*vec1 + t2*vec2)




############## DEFINE EDGE FLOW DIRECTION ##############


# use average direction of starting edges
avg_direction = define_average_direction(starting_edges)


# delete old faces
bmesh.ops.delete(bm, geom=faces_selected, context="FACES")


############## BUILD TRIANGLES #########################

# fill the queue with starting edges
queue = []
for i in range(len(starting_edges)):
    queue.append(starting_edges[i])

# build triangles
while len(queue) > 0:
    edge = queue.pop(0)

    if len(edge.link_faces) >= 2:
        continue

    # find the best particle
    best_particle = None
    best_particle_index = -1
    best_weight = 1000000000

    for i in range(len(particles)):
        # calculate weight
        dist1 = abs((edge.verts[0].co - particles[i]).length - avg_length)
        dist2 = abs((edge.verts[1].co - particles[i]).length - avg_length)
        weight = dist1 + dist2

        if weight < best_weight:
            best_weight = weight
            best_particle = particles[i]
            best_particle_index = i

    # find best vertex
    best_vertex = None
    best_vertex_index = -1
    best_weight = 1000000000

    for i in range(len(bm.verts)):
        # calculate weight
        dist1 = abs((edge.verts[0].co - bm.verts[i].co).length - avg_length)
        dist2 = abs((edge.verts[1].co - bm.verts[i].co).length - avg_length)
        weight = dist1 + dist2

        if weight < best_weight:
            best_weight = weight
            best_vertex = bm.verts[i]
            best_vertex_index = i


    new_vert = None
    # check if best vertex is good enough
    if best_weight < avg_length/2:    
        new_vert = best_vertex
    else:
        new_vert = bm.verts.new(best_particle)

    # create new face
    new_face = bm.faces.new((edge.verts[0], edge.verts[1], new_vert))

    bm.faces.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.verts.ensure_lookup_table()



    




############## END #####################################


sys.path.remove(syspath)
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




