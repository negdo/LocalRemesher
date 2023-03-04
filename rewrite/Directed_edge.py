import numpy as np

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
    