import numpy as np

def get_intersection(r1, q1, r2, q2):
    # find a point that is closest intersection of two lines in 3D
    # https://math.stackexchange.com/questions/2213165/find-shortest-distance-between-lines-in-3d
    # r1 and q2 are points on the first line
    # r2 and q2 are points on the second line
    # find distance between the lines

    # get vectors of lines
    e1 = q1 - r1
    e2 = q2 - r2

    # get normal vector of plane defined by lines
    n = np.cross(e1, e2)

    # if normal is zero, lines are parallel
    if np.all(n == 0):
        print("Lines are parallel")
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
        print("Lines do not intersect")
        return None

    # return middle point
    ratio = (1 - 2*abs(t1-0.5)) / (1 - 2*abs(t2-0.5))
    return p1 * ratio + p2 * (1-ratio)



# c = np.array([0, 1, 1])
# d = np.array([2, 1, 1])
# b = np.array([1, 0, 0])
# a = np.array([1.1, 2, 0.1])

# print(get_intersection(a, b, c, d))
