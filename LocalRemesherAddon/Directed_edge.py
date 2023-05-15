import numpy as np
from .other_utils import *
import math



class Directed_edge:

    def __init__(self, edge, vert):
        self.edge = edge
        self.vert = vert
        self.direction = vert.co - edge.other_vert(self.vert).co
        self.co = self.vert.co

    def original_edge_similarity(self, vec):
        # value of highest scalar product of edge with all neighbouring edges in original mesh
        # higher return is better

        highest_scalar = -1
        for edge in self.vert.link_edges:
            edge_vec = (edge.other_vert(self.vert).co - self.vert.co).normalized()
            scalar = np.dot(vec, edge_vec)
            highest_scalar = max(highest_scalar, scalar)

        if highest_scalar < 0:
            return 1
        else:
            return 1 - highest_scalar

        


class Weighted_edge:

    def __init__(self, start, end, avg_direction):
        self.start = start
        self.end = end
        self.weight = self.edge_similarity_weight(start, end, avg_direction)


    # calculates weight of how good would be edge between two vertices
    def edge_similarity_weight(self, edge1, edge2, avg_direction):
        # start and end are edges, look at their direction ...
        # higher weight is better

        # get vectors of the two edges
        edge1_vec = (edge1.edge.verts[0].co - edge1.edge.verts[1].co).normalized()
        edge2_vec = (edge2.edge.verts[0].co - edge2.edge.verts[1].co).normalized()

        # edge3 is the edge between the two vertices
        edge3_vec = edge1.vert.co - edge2.vert.co
        dist = np.linalg.norm(edge3_vec)
        edge3_vec = edge3_vec.normalized()

        # if distance is small, return 0
        if dist < 0.0001:
            return 0

        # get angle offset of edges - higher is more parallel
        angle = np.abs(np.dot(edge1_vec, edge3_vec)) + np.abs(np.dot(edge2_vec, edge3_vec))

        # get angle offset from average vector
        avg_angle_diff = np.abs(np.dot(avg_direction, edge3_vec))
        if avg_angle_diff < 0.5:
            # if less than 0.5, they are closer to perpendicular
            avg_angle_diff = 1 - avg_angle_diff
        avg_angle_diff = (avg_angle_diff - 0.5) * 5


        # get weight of similarity of original edges
        original_similarity = edge1.original_edge_similarity(edge3_vec) + edge2.original_edge_similarity(edge3_vec)

        # get weight
        weight = angle * avg_angle_diff**2 * original_similarity

        return weight
    
    def __lt__(self, other):
        return self.weight > other.weight
    

class Weighted_triangle:

    def __init__(self, triangle, avg_direction):
        self.triangle = triangle
        self.weights = self.get_dissolve_weight(triangle, avg_direction)
        self.max_weight = max(self.weights)
        self.max_edge = self.triangle.edges[self.weights.index(self.max_weight)]

    
    def get_dissolve_weight(self, triangle, avg_direction):
        # get weight of each edge to be disolved
        # higher weight means that edge will be disolved first
        
        weights = [0, 0, 0]
        
        for i in range(len(triangle.edges)):
            edge = triangle.edges[i]
            next_triangle = 0.01

            # check if the other face is triangle
            if len(edge.link_faces) == 2:
                other_face = edge.link_faces[0] if edge.link_faces[0] != triangle else edge.link_faces[1]
                if len(other_face.verts) == 3:
                    next_triangle = 1

                    # check if new quad would be convex
                    quad_verts = list(set(list(triangle.verts) + list(other_face.verts)))
                    if is_covex(quad_verts, verts_bool=True):
                        next_triangle = 10

            # check angle to average direction
            edge_vec = (edge.verts[0].co - edge.verts[1].co).normalized()
            angle = abs(np.dot(avg_direction, edge_vec))
            if angle < 0.5:
                angle = 1 - angle
            angle = 2*(angle - 0.5)
            # we want to dissolve edges that don't match average direction
            angle = 1 - angle

            # calculate edges length ratio, we want to dissolve longer edges more
            edge_length = edge.calc_length()
            other_edges_length = 0
            for other_edge in triangle.edges:
                if other_edge != edge:
                    other_edges_length += other_edge.calc_length()

            edge_length_ratio = edge_length / other_edges_length

            # calculate weight
            weights[i] = angle * next_triangle * edge_length_ratio

        return weights
    
    def __lt__(self, other):
        return self.max_weight > other.max_weight
    

class Weighted_face:

    def __init__(self, face):
        self.face = face
        self.weight = self.get_weight()

    def get_weight(self):
        # higher weight means face will be split first

        area = self.face.calc_area()
        if is_covex(self.face):
            convex = 1
        else:
            convex = 100
        
        odd = (1 - (len(self.face.verts) % 2)) * 10
        number_of_verts = len(self.face.verts)

        return area * convex * odd * number_of_verts
    
    def __lt__(self, other):
        return self.weight > other.weight







        
    