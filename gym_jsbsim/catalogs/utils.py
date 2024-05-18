import math


def reduce_reflex_angle_deg(angle):
    """ Given an angle in degrees, normalises in [-179, 180] """

    new_angle = angle % 360

    if new_angle > 180:
        new_angle -= 360

    return new_angle


"""replaced with normal distance function: dist between aircraft and centerline"""


def shortest_ac_dist(x, y, x1, y1, x2, y2):
    """
    Compute the shortest distance (in m) between aircraft coord (lat, lon) and the line between two points (lat1,lon1) and (lat2, lon2)
    >>> round(shortest_ac_dist(40.759809, -73.976264, 40.758492, -73.975105, 40.759752, -73.974215))
    160
    >>> round(shortest_ac_dist(40.759168, -73.976741, 40.758492, -73.975105, 40.759752, -73.974215))
    160
    """
    # print(x, y, x1, y1, x2, y2)
    # equation of line (lat1,lon1) -> (lat2, lon2): y = s*x + m
    # slope
    s = (y2 - y1) / (x2 - x1 + 0.00000000001)
    # find m: m = y - s*x
    m = y1 - s * x1
    # coeff of line equation ay + bx + c = 0
    a = -1
    b = s
    c = m
    # compute shortest distance between aircarft and the line = abs(a*x0 + b*y0 + c) / sqrt(a²+b²))
    s_ac_dist = math.fabs(a * x + b * y + c) / math.sqrt(a ** 2 + b ** 2)
    # print("s_ac_dist", s_ac_dist)
    return s_ac_dist
