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

def is_on_inside(point, edge):
    # returns False, if point is on the inside of face that is connected to edge
    if len(edge.link_faces) != 1:
        return False
    
    edge.link_faces[0].normal_update()
    plane1_normal = edge.link_faces[0].normal.normalized()
    plane2_normal = (edge.verts[1].co - edge.verts[0].co).normalized()
    center = (edge.verts[1].co + edge.verts[0].co) / 2

    line1, line2 = mathutils.geometry.intersect_plane_plane(center, plane1_normal, center, plane2_normal)
    line2 = line2.normalized() * edge.calc_length() + line1

    if line1 != None:
        projected, _ = mathutils.geometry.intersect_point_line(point, line1, line2)
        t = (projected - center).length

        if t <= 0.01:
            return False
    
    return True

def get_particle_weight(particle, edge, avg_length, mode="particle"):
    dist1 = abs((edge.verts[0].co - particle).length - avg_length)
    dist2 = abs((edge.verts[1].co - particle).length - avg_length)

    if mode == "vertex":
        # distance from center of edge - pick the closest one
        center = (edge.verts[0].co + edge.verts[1].co) / 2
        dist3 = abs((center - particle).length - avg_length)

        return dist1/avg_length + dist2/avg_length + 10*dist3/avg_length


    return dist1/avg_length + dist2/avg_length




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

bm_triangulated.faces.ensure_lookup_table()

for i in range(len(particles_per_face)):
    for j in range(particles_per_face[i]):
        # sample random point on triangle
        # vectors of triangle edges
        vec1 = bm_triangulated.faces[i].verts[1].co - bm_triangulated.faces[i].verts[0].co
        vec2 = bm_triangulated.faces[i].verts[2].co - bm_triangulated.faces[i].verts[0].co

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
bm.verts.ensure_lookup_table()
bm.edges.ensure_lookup_table()
bm.faces.ensure_lookup_table()

avaliable_verts = [vert for vert in bm.verts if vert.select == True]
side_edges = [edge for edge in bm.edges if edge.select == True]

avg_length = sum([edge.calc_length() for edge in side_edges]) / len(side_edges)
print("Average length: " + str(avg_length))

# remove particles that are too close to edges
radius = avg_length / 3
for side_edge in side_edges:
    vert1 = side_edge.verts[0].co
    vert2 = side_edge.verts[1].co
    for particle in particles.copy():
        point, t = mathutils.geometry.intersect_point_line(particle, vert1, vert2)
        distance = (point - particle).length
        
        # distance to edge
        if t >= -0.2 and t <= 1.2:
            if distance < radius:
                particles.remove(particle)


############## BUILD TRIANGLES #########################




# fill the queue with starting edges
queue = []
for i in range(len(side_edges)):
    queue.append(side_edges[i])

max_faces = 500
# build triangles
while len(queue) > 0 and max_faces > 0:
    edge = queue.pop(0)
    print("Edge: " + str(edge.verts[0].co) + " " + str(edge.verts[1].co))

    if len(edge.link_faces) >= 2:
        continue

    # find the best particle
    best_particle = None
    best_weight_particles = 10000

    for i in range(len(particles)):
        if not is_on_inside(particles[i], edge):
            continue

        weight = get_particle_weight(particles[i], edge, avg_length)
        if weight < best_weight_particles:
            best_weight_particles = weight
            best_particle = particles[i]

    # find best vertex
    best_vertex = None
    best_weight = 1000000000
    
    for i in range(len(avaliable_verts)):
        
        current_vert = avaliable_verts[i]
        if current_vert == edge.verts[0] or current_vert == edge.verts[1]:
            continue

        vertex_usable = False
        for used_edge in current_vert.link_edges:
            if len(used_edge.link_faces) < 2:
                vertex_usable = True
                break
        if vertex_usable == False:
            continue

        link_verts1 = [edge1.other_vert(edge.verts[0]) for edge1 in edge.verts[0].link_edges]
        link_verts2 = [edge2.other_vert(edge.verts[1]) for edge2 in edge.verts[1].link_edges]

        if current_vert in link_verts1 and current_vert in link_verts2:
            # if they share a face, they are not usable otherwise this is the best vertex
            if len([face for face in current_vert.link_faces if face in edge.verts[0].link_faces and face in edge.verts[1].link_faces]) == 0:
                best_weight = 0
                best_vertex = current_vert
                break
            else:
                continue

        if current_vert in link_verts1:
            connecting_edge = [edge1 for edge1 in edge.verts[0].link_edges if edge1.other_vert(edge.verts[0]) == current_vert][0]
            if len(connecting_edge.link_faces) >= 2:
                continue
        
        if current_vert in link_verts2:
            connecting_edge = [edge2 for edge2 in edge.verts[1].link_edges if edge2.other_vert(edge.verts[1]) == current_vert][0]
            if len(connecting_edge.link_faces) >= 2:
                continue

        # check if angle between normals is too small
        if not is_on_inside(current_vert.co, edge):
            continue 

        link_faces1 = [face for face in current_vert.link_faces if face in edge.verts[0].link_faces]
        link_faces2 = [face for face in current_vert.link_faces if face in edge.verts[1].link_faces]

        if link_faces1 != []:
            if current_vert not in link_verts1:
                continue
        if link_faces2 != []:
            if current_vert not in link_verts2:
                continue

        # calculate weight
        weight = get_particle_weight(current_vert.co, edge, avg_length)
        if weight < best_weight:
            best_weight = weight
            best_vertex = current_vert

    new_vert = None
    # check if best vertex is good enough
    if (best_weight < 1 or best_weight_particles >= best_weight) and best_vertex != None:    
        new_vert = best_vertex
    else:
        if best_particle == None:
            continue
        new_vert = bm.verts.new(best_particle)
        avaliable_verts.append(new_vert)

    # create new face
    new_face = bm.faces.new((edge.verts[0], edge.verts[1], new_vert))
    new_face.select = True
    max_faces -= 1

    bm.faces.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.verts.ensure_lookup_table()

    for edge in new_face.edges:
        if len(edge.link_faces) < 2:
            queue.append(edge)

    
    # remove particles around new face
    radius = avg_length/2

    for side_edge in new_face.edges:
        vert1 = side_edge.verts[0].co
        vert2 = side_edge.verts[1].co
        for particle in particles.copy():
            point, t = mathutils.geometry.intersect_point_line(particle, vert1, vert2)
            distance = (point - particle).length
            
            # distance to edge
            if t >= -0.2 and t <= 1.2:
                if distance < radius:
                    particles.remove(particle)
    


for particle in particles:
    bm.verts.new(particle)


edges = set(edge for face in bm.faces for edge in face.edges if face.select)
for side_edge in side_edges:
    if side_edge in edges:
        edges.remove(side_edge)
edges = list(edges)



############## IPROVE MESH ###########################



    




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




