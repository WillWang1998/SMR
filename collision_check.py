def collision_line2line(x1, y1, x2, y2, x3, y3, x4, y4):
    # x1y1 and x2y2 are 2 endpoints on line 1, x3y3 and x4y4 are 2 endpoints on line 2
    t1 = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));

    t2 = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
    if 0 <= t1 <= 1 and 0 <= t2 <= 1:
        return True  # collision happened
    else:
        return False  # no collision


def collision_point2polygon(x1, y1, polygon):
    return True


def collision_line2polygon(x1, y1, x2, y2, polygon):
    return True
