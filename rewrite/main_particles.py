import os
import pathlib
import sys

import bmesh
import bpy
import mathutils

# get file location
syspath = os.path.abspath(__file__)
syspath = pathlib.Path(syspath).parent.absolute()
syspath = pathlib.Path(syspath).parent.absolute()
syspath = str(syspath) + "\\rewrite"
sys.path.append(syspath)

from selection_utils import *
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

    raw_dist1 = (edge.verts[0].co - particle).length
    raw_dist2 = (edge.verts[1].co - particle).length

    dist1 = abs(raw_dist1 - avg_length)
    dist2 = abs(raw_dist2 - avg_length)

    if mode == "vertex":
        # distance from center of edge - pick the closest one
        center = (edge.verts[0].co + edge.verts[1].co) / 2
        dist3 = abs((center - particle).length - avg_length)

        return (dist1 / avg_length + dist2 / avg_length + 10 * dist3 / avg_length) * ((raw_dist1 + raw_dist2) / 2 / avg_length)

    return dist1 / avg_length + dist2 / avg_length


############## INIT #################################

# select more and less to get inner unselected faces
bpy.ops.mesh.select_more()
bpy.ops.mesh.select_less()

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
particles_per_face = np.random.multinomial(num_particles, weights / np.sum(weights))
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
        particles.append(bm_triangulated.faces[i].verts[0].co + t1 * vec1 + t2 * vec2)

# magnet particles to sharp edges
sharp_radius = avg_length * 3

edges_sharpnes = {}
for edge in bm.edges:
    if edge.select == True:
        angle = edge.calc_face_angle(None)
        if angle is None:
            angle = 0
        if angle > math.pi / 6:
            distance = min(1, angle / math.pi - 1 / 6) * sharp_radius
            edges_sharpnes[edge] = distance

for i in range(len(particles)):
    for edge in edges_sharpnes:
        new_point, _ = mathutils.geometry.intersect_point_line(particles[i], edge.verts[0].co, edge.verts[1].co)
        dist = (new_point - particles[i]).length
        t = (new_point - edge.verts[1].co).length + (new_point - edge.verts[0].co).length > 1.1 * edge.calc_length()
        if dist < edges_sharpnes[edge] and t:
            particles[i] = new_point
            break


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
            if len([face for face in current_vert.link_faces if
                    face in edge.verts[0].link_faces and face in edge.verts[1].link_faces]) == 0:
                best_weight = 0
                best_vertex = current_vert
                break
            else:
                continue

        if current_vert in link_verts1:
            connecting_edge = \
                [edge1 for edge1 in edge.verts[0].link_edges if edge1.other_vert(edge.verts[0]) == current_vert][0]
            if len(connecting_edge.link_faces) >= 2:
                continue

        if current_vert in link_verts2:
            connecting_edge = \
                [edge2 for edge2 in edge.verts[1].link_edges if edge2.other_vert(edge.verts[1]) == current_vert][0]
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
    radius = avg_length / 2

    # remove particles too close to the face
    for particle in particles.copy():
        closes_point = mathutils.geometry.closest_point_on_tri(particle, new_face.verts[0].co, new_face.verts[1].co, new_face.verts[2].co)
        distance = (closes_point - particle).length
        if distance < radius:
            particles.remove(particle)


edges = set(edge for face in bm.faces for edge in face.edges if face.select)
for side_edge in side_edges:
    if side_edge in edges:
        edges.remove(side_edge)
edges = list(edges)

bm.faces.ensure_lookup_table()
bm.edges.ensure_lookup_table()
bm.verts.ensure_lookup_table()
bm.faces.index_update()
bm.edges.index_update()
bm.verts.index_update()


############## IPROVE MESH ###########################

def get_angle_between_edges(edge1, edge2, common_vert):
    vec1 = (common_vert.co - edge1.other_vert(common_vert).co).normalized()
    vec2 = (common_vert.co - edge2.other_vert(common_vert).co).normalized()
    return abs(vec1.angle(vec2) % math.pi)


def get_edge_disolve_weight(edge, avg_direction):
    for face in edge.link_faces:
        if len(face.verts) != 3:
            return 100000000

    if len(edge.link_faces) != 2:
        return 100000000

    vec = (edge.verts[1].co - edge.verts[0].co).normalized()
    angle = vec.angle(avg_direction)
    # weight is biggest when angle to average direction is 45 degrees
    angle = abs(abs(abs(angle) - math.pi / 2) - math.pi / 4)

    # get size of angles of new face
    face1 = edge.link_faces[0]
    face2 = edge.link_faces[1]

    corner1_edge1 = [edge1 for edge1 in edge.verts[0].link_edges if edge1 != edge and edge1 in face1.edges][0]
    corner1_edge2 = [edge2 for edge2 in edge.verts[0].link_edges if edge2 != edge and edge2 in face2.edges][0]

    corner2_edge1 = [edge1 for edge1 in edge.verts[1].link_edges if edge1 != edge and edge1 in face1.edges][0]
    corner2_edge2 = [edge2 for edge2 in edge.verts[1].link_edges if edge2 != edge and edge2 in face2.edges][0]

    angle11 = get_angle_between_edges(corner1_edge1, edge, edge.verts[0])
    angle12 = get_angle_between_edges(corner1_edge2, edge, edge.verts[0])

    angle21 = get_angle_between_edges(corner2_edge1, edge, edge.verts[1])
    angle22 = get_angle_between_edges(corner2_edge2, edge, edge.verts[1])

    angle1 = angle11 + angle12
    angle2 = angle21 + angle22

    if angle1 > math.pi or angle2 > math.pi:
        return 100000000

    # weight is lowest when angles are 90 degrees
    angle1 = abs(abs(angle1) - math.pi / 2)
    angle2 = abs(abs(angle2) - math.pi / 2)

    # faces should be planar
    face1 = edge.link_faces[0]
    face2 = edge.link_faces[1]

    face1.normal_update()
    face2.normal_update()

    normal1 = face1.normal
    normal2 = face2.normal

    if normal1.length == 0 or normal2.length == 0:
        return 100000000

    normal_diff = normal1.angle(normal2)

    return angle + angle1 + angle2 + normal_diff


def iterativly_disolve_edges(triangles, avg_direction):
    while len(triangles) > 0:
        current_face = triangles.pop(0)
        if not current_face.is_valid or len(current_face.verts) != 3:
            continue

        # get edge with highest weight
        best_edge = None
        best_weight = 100000000
        for edge in current_face.edges:
            weight = get_edge_disolve_weight(edge, avg_direction)
            if weight < best_weight:
                best_weight = weight
                best_edge = edge

        if best_weight == 100000000:
            continue

        link_faces = set()
        for face in best_edge.link_faces:
            for edge in face.edges:
                for link_face in edge.link_faces:
                    link_faces.add(link_face)

        # disolve edge
        bmesh.ops.dissolve_edges(bm, edges=[best_edge])

        # add link faces to queue
        for face in link_faces:
            if face.is_valid and len(face.verts) == 3 and face.select:
                triangles.append(face)


# recalculate normals
selected_faces = list(set(face for vert in bm.verts for face in vert.link_faces if vert.select))
bmesh.ops.recalc_face_normals(bm, faces=selected_faces)

# detirangulate
selection = [face for face in bm.faces if face.select and len(face.verts) == 3]
while len(selection) > 0:
    if selection[-1].is_valid and selection[-1].select:
        iterativly_disolve_edges([selection.pop()], avg_direction)
    else:
        selection.pop()

selected_verts = [vert for vert in bm.verts if vert.select]

# smooth the vertices
bmesh.ops.smooth_vert(bm, verts=selected_verts, factor=0.5)

selected_faces = list(set(face for vert in bm.verts for face in vert.link_faces if vert.select))
# recalculate normals
bmesh.ops.recalc_face_normals(bm, faces=selected_faces)

# project vertices to original mesh uing interpolation
vertices = [vert for vert in bm.verts if vert.select]
projection_tree = preprate_bm_for_projection(bm_proj2, subdiv=1, delete_faces=False)
for vert in vertices:
    vert.co = project_point_to_faces(projection_tree, vert.co, bm_proj2)


bm.edges.ensure_lookup_table()
bm.verts.ensure_lookup_table()
bm.faces.ensure_lookup_table()

# update selection (select faces that have all vertices selected)
for face in bm.faces:
    if face.is_valid:
        selected = True
        for vert in face.verts:
            if not vert.select:
                selected = False
                break
        face.select = selected


# look at pairs of quads and potentially change the splitting line
vertices = [vert for vert in bm.verts if vert.select and vert.is_valid]
improve_edge_flow_direction(bm, vertices, avg_direction)

# relax vertices to get more even mesh
# vertices = [v for v in vertices if v.is_valid]
# relax_vertices(vertices, 10, 10)


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
