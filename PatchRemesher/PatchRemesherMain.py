import bpy
import bmesh
import mathutils
from queue import PriorityQueue

# this algorithm should get a part of 3d model and make patches out of it
# patches should then be individually remeshed, but take into account the neighbouring patches

SHARP_EDGE_ANGLE = 0.5  # edges with angle greater than this will be marked as sharp


class Patch:
    # list of faces and border edges
    def __init__(self, face):
        self.faces = []
        self.border_edges = []
        self.patch_search(face)
        self.faces = list(self.faces)
        self.border_edges = list(self.border_edges)
        self.average_normal = self.get_average_normal()
        self.u, self.v = self.get_u_v_vectors()
        self.is_2D_convex = self.is_border_convex()

    def patch_search(self, face):
        # recursively search for all faces that are connected to the face and add them to patch_faces
        # also add border edges to patch_border_edges
        self.faces.append(face)
        for edge in face.edges:
            if edge.seam:
                self.border_edges.append(edge)
            else:
                for new_face in edge.link_faces:
                    if new_face not in self.faces:
                        self.patch_search(new_face)

    def get_average_normal(self):
        normal = mathutils.Vector((0, 0, 0))
        for face in self.faces:
            normal += face.normal
        return normal / len(self.faces)

    def get_average_normal_diviation(self):
        normal = self.get_average_normal()
        diviation = 0
        for face in self.faces:
            diviation += (face.normal - normal).length
        return diviation / len(self.faces)

    def get_u_v_vectors(self):
        # returns two vectors of the plane that are perpendicular to self.average_normal and eachother
        u = mathutils.Vector((0, 0, 1))
        v = mathutils.Vector((0, 1, 0))
        if abs(self.average_normal.dot(u)) > abs(self.average_normal.dot(v)):
            u, v = v, u
        u = u - self.average_normal.dot(u) * self.average_normal
        v = v - self.average_normal.dot(v) * self.average_normal
        u.normalize()
        v.normalize()

        return u, v

    def project_vertex_on_plane(self, vertex):
        # project vertex on plane with self.average_normal
        # returns (u, v) coordinates
        projected_vertex = mathutils.geometry.intersect_line_plane(vertex.co, vertex.co + self.average_normal, (0, 0, 0), self.average_normal)
        u = projected_vertex.dot(self.u)
        v = projected_vertex.dot(self.v)
        return mathutils.Vector([u, v]).freeze()

    def is_border_convex(self):
        # returns true if all border vertices are convex on 2D plane
        border_verts = list(set(self.project_vertex_on_plane(vert) for edge in self.border_edges for vert in edge.verts))

        hull = mathutils.geometry.convex_hull_2d(border_verts)
        if len(hull) != len(border_verts):
            return False
        return True

    def split_patch(self):
        # finds two porder points, where at least one is not in convex hull
        border_verts = list(set(vert for edge in self.border_edges for vert in edge.verts))
        border_verts_projected = [self.project_vertex_on_plane(vert) for vert in border_verts]

        hull = mathutils.geometry.convex_hull_2d(border_verts_projected)
        if len(hull) == len(border_verts_projected):
            return False
        # v1 is atleast 2 points away from v2
        min_dist = 100000
        shortest_path = False

        # TODO priority queue
        # TODO ravni edgi so bolši
        # TODO za povezovt so boljši edgi k so povezani samo na en seam edge



        for v1 in range(len(border_verts_projected) - 1):
            for v2 in range(v1 + 1, len(border_verts_projected)):
                if border_verts_projected[v1] in hull and border_verts_projected[v2] in hull:
                    continue
                unprojected1 = self.unproject_vertex(border_verts_projected[v1], border_verts)
                unprojected2 = self.unproject_vertex(border_verts_projected[v2], border_verts)
                if unprojected1 is None or unprojected2 is None:
                    continue
                if unprojected1 == unprojected2:
                    continue
                linked_verts_seam = [edge.other_vert(unprojected1) for edge in unprojected1.link_edges if edge.seam]
                if unprojected2 in linked_verts_seam:
                    continue
                found_path = self.get_shortest_path(unprojected1, unprojected2)
                if found_path is None:
                    continue

                if found_path[1] < min_dist:
                    min_dist = found_path[1]
                    shortest_path = found_path[0]
        if min_dist == 100000:
            return False

        return shortest_path

    def unproject_vertex(self, vertex, vert_list):
        found_verts = [vert for vert in vert_list if self.project_vertex_on_plane(vert) == vertex]
        if len(found_verts) == 0:
            return None
        return found_verts[0]

    def get_shortest_path(self, v1, v2):
        # shortest path between two vertices on the patch
        # returns list of edges, sum length of edges
        # BFS

        queue = PriorityQueue()
        queue.put((0, (v1, [], 0)))

        path = []
        while not queue.empty():
            vertex, path, depth = queue.get()
            if depth > 5:
                return None
            if vertex == v2:
                break
            conneced_edges = [edge for edge in vertex.link_edges if edge not in path]
            conneced_edges.sort(key=lambda edge: edge.calc_length())
            for edge in conneced_edges:
                if edge.seam:
                    continue
                if len(edge.link_faces) != 2:
                    continue
                if edge.link_faces[0] not in self.faces or edge.link_faces[1] not in self.faces:
                    continue
                queue.put((0, (edge.other_vert(vertex), path + [edge], depth+1)))
        if len(path) == 0:
            return None
        return path, sum(e.calc_length() for e in path)













############## INIT #################################
# select more and less to delete single-face islands
bpy.ops.mesh.select_more()
bpy.ops.mesh.select_less()

# to object mode
bpy.ops.object.mode_set(mode='OBJECT')

# init bmesh objects
me = bpy.context.object.data
bm = bmesh.new()
bm.from_mesh(me)

############### SELECTION ##########################
# deselect all edges that are not connected to a selected face
selected_edges = []
for e in bm.edges:
    if e.select:
        face_selected = False
        for f in e.link_faces:
            if f.select:
                face_selected = True
                break
        if face_selected:
            selected_edges.append(e)
        else:
            e.select = False

############## SETTING SIMPLE SEAMS ####################
# Outer edges
# set UV seam on all edges that are connected to a face that is not selected
for e in selected_edges:
    if e.select:
        for f in e.link_faces:
            if not f.select:
                e.seam = True
                break

# Sharp edges
# set UV seam on all edges that have an angle greater than SHARP_EDGE_ANGLE
for e in selected_edges:
    if e.select:
        if e.calc_face_angle(2) > SHARP_EDGE_ANGLE:
            e.seam = True

############## SUBDIVIDE PATCHES #####################

# get list of patches subdivided by seams
selected_faces = [f for f in bm.faces if f.select]
patches = []
while len(selected_faces) > 0:
    # get first face and find its patch
    f = selected_faces[0]
    selected_faces.remove(f)
    patch = Patch(f)
    patches.append(patch)
    for f in patch.faces:
        if f in selected_faces:
            selected_faces.remove(f)

n_splits = 5
while n_splits > 0:
    for patch in patches.copy():
        if not patch.is_2D_convex:
            path = patch.split_patch()
            print(path)
            if path != False:
                patches.remove(patch)

                split_edge = path[0]

                for edge in path:
                    if not edge.seam:
                        split_edge = edge
                        edge.seam = True

                patch = Patch(split_edge.link_faces[0])
                patches.append(patch)
                patch = Patch(split_edge.link_faces[1])
                patches.append(patch)

    print(n_splits)
    n_splits -= 1







############## END #####################################

# unvrap selected faces
bm.to_mesh(me)
bm.free()
bpy.ops.object.mode_set(mode='EDIT')

