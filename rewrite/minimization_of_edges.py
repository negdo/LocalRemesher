# Koda, k jo bom mogoče še rabu
# zamenja priority queue z linear sum assignment
# ne dela vredu, ker je lahko iz enga vertexa več edgov


# matrix of weights for each connection, higher is better
weights = - np.ones((len(edges_to_connect), len(edges_to_connect)))
edge_dict = {}

for i in range(len(edges_to_connect)-1):
    for j in range(i+1, len(edges_to_connect)):

        # calculate weight
        weight = edge_similarity_weight(edges_to_connect[i], edges_to_connect[j], avg_direction)

        # check edge already exists in original mesh
        if edges_to_connect[j].vert not in [e.other_vert(edges_to_connect[i].vert) for e in edges_to_connect[i].vert.link_edges]:

            # add weight to matrix
            weights[i][j] = weight
            edge_dict[(i,j)] = (edges_to_connect[i], edges_to_connect[j])


# get optimal connections - one element in each row and column
row_ind, col_ind = linear_sum_assignment(weights, maximize=True)

print("")
print("Weights:")
for i in range(len(weights)):
    print(weights[i])


print("")
print("Row indices:")
print(row_ind)
print("Column indices:")
print(col_ind)
print("weights:")
print(weights[row_ind, col_ind])

for i in range(len(row_ind)):
    # check if weight is -1
    if weights[row_ind[i]][col_ind[i]] != -1:
        if (row_ind[i], col_ind[i]) in edge_dict:
            edge1, edge2 = edge_dict[(row_ind[i], col_ind[i])]
        elif (col_ind[i], row_ind[i]) in edge_dict:
            edge1, edge2 = edge_dict[(col_ind[i], row_ind[i])]
        else:
            print("Error")

        connecting_vertices.append([edge1.vert, edge2.vert])

print([[e1.index, e2.index] for e1, e2 in connecting_vertices])
