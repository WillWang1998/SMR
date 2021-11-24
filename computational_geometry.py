import math


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Point((self.x + other.x), (self.x + other.y))

    def __sub__(self, other):
        return Point((self.x - other.x), (self.x - other.y))

    def __mul__(self, number):
        return Point(self.x * number, self.y * number)

    def __truediv__(self, number):
        return Point(self.x / number, self.y / number)

    def rotate(self, angle):
        return Point(self.x * math.cos(angle) - self.y * math.sin(angle),
                     self.y * math.cos(angle) + self.x * math.sin(angle))


class Arc:
    def __init__(self, point_a: Point, theta: float, r: float, delta: float, direction: int):
        self.point_a = point_a
        self.direction = direction
        self.r = r
        if direction == 0:
            self.center = Point(point_a.x - r * math.sin(theta), point_a.y + r * math.cos(theta))
            point_a_relatively = self.point_a - self.center
            point_b_relatively = point_a_relatively.rotate(delta / r)
            self.point_b = point_b_relatively + self.center
            self.degree_a = (theta - math.pi / 2) % (2 * math.pi)
            self.degree_b = (delta / r + theta - math.pi / 2) % (2 * math.pi)
        else:
            self.center = Point(point_a.x + r * math.sin(theta), point_a.y - r * math.cos(theta))
            point_a_relatively = self.point_a - self.center
            point_b_relatively = point_a_relatively.rotate(-delta / r)
            self.point_b = point_b_relatively + self.center
            self.degree_a = (theta + math.pi / 2) % (2 * math.pi)
            self.degree_b = (- delta / r + theta + math.pi / 2) % (2 * math.pi)

    def point_on_arc(self, point: Point) -> bool:  # The point is on the circle
        point_relatively = point - self.center
        if point_relatively.y < 0:
            degree = math.pi + math.acos(- point_relatively.x / self.r)
        else:
            degree = math.acos(point_relatively.x / self.r)

        if self.degree_a < self.degree_b:
            return self.degree_a <= degree <= self.degree_b
        else:
            return self.degree_a <= degree or degree <= self.degree_b


class Segment:
    def __init__(self, point_a: Point, point_b: Point):
        self.point_a = point_a
        self.point_b = point_b

    def has_intersect_point_with_right_forward_ray_from_point(self, point: Point) -> bool:
        min_y = min(self.point_a.y, self.point_b.y)
        max_y = max(self.point_a.y, self.point_b.y)
        if max_y < 0 or min_y > 0:
            return False
        else:
            if self.point_a.x - self.point_b.x != 0:
                k = (self.point_a.y - self.point_b.y) / (self.point_a.x - self.point_b.x)
                b = self.point_a.y - k * self.point_a.x
                x = - b / k
                if x > point.x:
                    return True
                else:
                    return False
            else:
                return self.point_a.x > point.x

    def point_on_segment(self, point: Point) -> bool:  # The point is on the line
        min_x = min(self.point_a.x, self.point_b.x)
        max_x = max(self.point_a.x, self.point_b.x)
        min_y = min(self.point_a.y, self.point_b.y)
        max_y = max(self.point_a.y, self.point_b.y)
        return min_x <= point.x <= max_x and min_y <= point.y <= max_y

    def has_intersect_point_with_arc(self, arc: Arc) -> bool:
        point_a = self.point_a - arc.center
        point_b = self.point_b - arc.center
        if point_a.x == point_b.x:
            if abs(point_a.x) > arc.r:
                return False
            elif abs(point_a.x) == arc.r:
                intersect_point = arc.center + Point(point_a.x, 0)
                return self.point_on_segment(intersect_point) and arc.point_on_arc(intersect_point)
            else:
                intersect_point_a = arc.center + Point(point_a.x, math.sqrt(arc.r ** 2 - point_a.x ** 2))
                intersect_point_b = arc.center + Point(point_a.x, -math.sqrt(arc.r ** 2 - point_a.x ** 2))
                return (self.point_on_segment(intersect_point_a) and arc.point_on_arc(intersect_point_a)) or \
                       (self.point_on_segment(intersect_point_b) and arc.point_on_arc(intersect_point_b))
        else:
            k = (point_a.y - point_b.y) / (point_a.x - point_b.x)
            b = point_a.y - k * point_a.x
            if (k ** 2 + 1) * (arc.r ** 2) - b ** 2 < 0:
                return False
            elif (k ** 2 + 1) * (arc.r ** 2) - b ** 2 == 0:
                x = - k * b / (k ** 2 + 1)
                y = k * x + b
                intersect_point = arc.center + Point(x, y)
                return self.point_on_segment(intersect_point) and arc.point_on_arc(intersect_point)
            else:
                x_1 = (- 2 * k * b - 2 * math.sqrt((k ** 2 + 1) * (arc.r ** 2) - b ** 2)) / (2 * (k ** 2 + 1))
                x_2 = (- 2 * k * b + 2 * math.sqrt((k ** 2 + 1) * (arc.r ** 2) - b ** 2)) / (2 * (k ** 2 + 1))
                y_1 = k * x_1 + b
                y_2 = k * x_2 + b
                intersect_point_a = arc.center + Point(x_1, y_1)
                intersect_point_b = arc.center + Point(x_2, y_2)
                return (self.point_on_segment(intersect_point_a) and arc.point_on_arc(intersect_point_a)) or \
                       (self.point_on_segment(intersect_point_b) and arc.point_on_arc(intersect_point_b))


class Polygon:
    def __init__(self, raw_points):
        self.points = []
        for raw_point in raw_points:
            self.points.append(Point(raw_point[0], raw_point[1]))
        self.segments = []
        for i in range(1, len(self.points)):
            self.segments.append(Segment(self.points[i - 1], self.points[i]))

    def intersect_with_arc(self, arc: Arc) -> bool:
        for segment in self.segments:
            if segment.has_intersect_point_with_arc(arc):
                return True
        return False

    def contains(self, point: Point) -> bool:
        intersect_cnt = 0
        for segment in self.segments:
            if segment.has_intersect_point_with_right_forward_ray_from_point(point):
                intersect_cnt += 1
        return intersect_cnt % 2 != 0


def distance_for_collision_checking(point_a: Point, point_b: Point) -> float:
    return math.sqrt((point_a.x - point_b.x) ** 2
                     + (point_a.y - point_b.y) ** 2)
