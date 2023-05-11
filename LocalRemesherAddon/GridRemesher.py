import bpy
import bmesh
from .selection_utils import *
from .loop_utils import *
from .building_utils import *
from .improving_utils import *


class GridRemesher(bpy.types.Operator):
    bl_idname = "localremesher.remesh_grid"
    bl_label = "Grid Remesher"
    bl_description = "Remeshes the selected part of the mesh using a grid method"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):

        ############## INIT #################################
        bpy.ops.object.mode_set(mode='OBJECT')
        me = bpy.context.object.data
        bm = bmesh.new()
        bm.from_mesh(me)
        bm_for_convex = bmesh.new()
        bm_for_convex.from_mesh(me)
        bm_proj = bmesh.new()
        bm_proj.from_mesh(me)
        bm_proj2 = bmesh.new()
        bm_proj2.from_mesh(me)

        ############## SELECTION ############################
        # get selected faces and their vertices for convex mesh
        faces_selected, verts_selected = get_selected(bm_for_convex)

        # get selected faces and their vertices
        faces_selected, verts_selected = get_selected(bm)

        # get starting edges, illegal edges and average length of starting edges
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

        if len(connecting_vertices) > 0:
            # use average direction of already defined loops
            avg_direction = (connecting_vertices[0][1].co - connecting_vertices[0][0].co).normalized()
        else:
            # use average direction of starting edges
            avg_direction = define_average_direction(starting_edges)

        # delete old faces
        bmesh.ops.delete(bm, geom=faces_selected, context="FACES")

        ############## CONNECTING OTHER VERTICES ###############
        # edges that are facing the avg_direction (or perpendicular) have higher priority
        # calculate weight for each combination of edges

        edges_to_connect = [edge for edge in starting_edges if edge not in used_edges]
        # sort edges_to_connect by index of edge
        edges_to_connect.sort(key=lambda x: x.edge.index)

        # init priority queue
        pq = []
        heapq.heapify(pq)

        for i in range(len(edges_to_connect) - 1):
            for j in range(i + 1, len(edges_to_connect)):
                # check edge already exists in original mesh
                if edges_to_connect[j].vert not in [e.other_vert(edges_to_connect[i].vert) for e in
                                                    edges_to_connect[i].vert.link_edges]:
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
        projection_tree = preprate_bm_for_projection(bm_proj, subdiv=0, delete_faces=True)

        # check intersection between each pair of lines
        for i in range(len(connecting_vertices) - 1):
            for j in range(i + 1, len(connecting_vertices)):
                # get intersection point
                intersect = get_intersection(connecting_vertices[i][0].co, connecting_vertices[i][1].co,
                                             connecting_vertices[j][0].co, connecting_vertices[j][1].co)

                if intersect is not None:
                    # project the point on original mesh
                    intersect = project_point_to_faces(projection_tree, intersect, bm_proj)
                    if intersect is None:
                        continue

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

            for j in range(len(path) - 1):
                # check if edge would be too long, in that case split it
                if (path[j].co - path[j + 1].co).length > avg_length * 2:
                    # split edge
                    new_vert = bm.verts.new((path[j].co + path[j + 1].co) / 2)
                    edge1 = bm.edges.new((path[j], new_vert))
                    edge2 = bm.edges.new((new_vert, path[j + 1]))
                    avg_new_edge_length += (new_vert.co - path[j + 1].co).length + (path[j].co - new_vert.co).length
                    vertices.add(path[j])
                    vertices.add(new_vert)
                    vertices.add(path[j + 1])
                    edges.append(edge1)
                    edges.append(edge2)
                else:
                    try:
                        edge = bm.edges.new((path[j], path[j + 1]))
                        avg_new_edge_length += (path[j].co - path[j + 1].co).length
                        vertices.add(path[j])
                        edges.append(edge)
                    except:
                        print("ERROR: Edge already exists")

            vertices.add(path[-1])
            vertices.add(path[0])
            outside_vertices.add(path[-1])
            outside_vertices.add(path[0])

        vertices = list(vertices)
        avg_new_edge_length /= len(connecting_vertices)

        # merge vertices by distance
        bmesh.ops.remove_doubles(bm, verts=vertices, dist=avg_length / 4)

        bm.edges.ensure_lookup_table()
        bm.verts.ensure_lookup_table()

        # remove vertices that were merged from vertices list
        vertices = [v for v in vertices if v.is_valid and v not in outside_vertices]

        edges = list(set(edge for vert in vertices for edge in vert.link_edges))

        ############## IPROVE MESH ###########################
        # find all triangles, empty and already filled triangles
        triangles, triangles_faces = get_triangles(edges)

        # fill triangles list with new faces
        for triangle in triangles:
            triangles_faces.append(bm.faces.new(triangle))

        bm.edges.ensure_lookup_table()
        bm.verts.ensure_lookup_table()
        bm.faces.ensure_lookup_table()

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

        bm.edges.ensure_lookup_table()
        bm.verts.ensure_lookup_table()
        bm.faces.ensure_lookup_table()

        # update edges
        edges = [e for e in edges if e.is_valid]

        print("### FILL FACE HOLES ###")
        bm.verts.index_update()

        # Create new faces n-gons
        created_faces = get_faces(edges, bm)

        bm.faces.ensure_lookup_table()
        bm.faces.index_update()

        # add created faces edges to edges list
        for face in created_faces:
            for edge in face.edges:
                edges.append(edge)
                edge.select = True

        edges = list(set(edges))

        # delete edges that are not part of 2 faces
        for edge in edges:
            if len(edge.link_faces) < 2:
                bmesh.ops.delete(bm, geom=[edge], context="EDGES")

        bm.edges.ensure_lookup_table()
        bm.verts.ensure_lookup_table()
        bm.faces.ensure_lookup_table()
        created_faces = [f for f in created_faces if f.is_valid]

        print("### CONVERT N-GONS TO QUADS ###")

        # convert n-gons to quads
        created_faces = subdivide_faces_to_quads(bm, created_faces, avg_direction)

        for face in created_faces:
            face.select = True

        # get vertices
        vertices = list(set(vert for face in created_faces for vert in face.verts if vert.is_valid))

        # look at pairs of quads and potentially change the splitting line
        improve_edge_flow_direction(bm, vertices, avg_direction)

        # relax vertices to get more even mesh
        vertices = [v for v in vertices if v.is_valid]
        relax_vertices(vertices, 10, 10)

        # straiten loops
        # TODO

        # project vertices to original mesh uing interpolation
        projection_tree = preprate_bm_for_projection(bm_proj2, subdiv=1, delete_faces=False)
        for vert in vertices:
            vert.co = project_point_to_faces(projection_tree, vert.co, bm_proj2)

        # recalculate normals
        bmesh.ops.recalc_face_normals(bm, faces=[face for face in created_faces if face.is_valid])

        ############## END #####################################
        bm.to_mesh(me)
        bm.free()
        bm_for_convex.free()
        bpy.ops.object.mode_set(mode='EDIT')

        return {"FINISHED"}


if __name__ == "__main__":
    bpy.utils.register_class(GridRemesher)
    bpy.ops.localremesher.remesh_grid()
