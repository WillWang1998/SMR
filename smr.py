import math
import numpy as np
from computational_geometry import Polygon, Point, Arc, distance_for_collision_checking, Segment
from sklearn.neighbors import KDTree


class Action:
    def __init__(self, delta_average, r_average, delta_standard_deviation, r_standard_deviation):
        self.delta_average = delta_average
        self.r_average = r_average
        self.delta_standard_deviation = delta_standard_deviation
        self.r_standard_deviation = r_standard_deviation

    def sample(self):
        delta = np.random.normal(self.delta_average, self.delta_standard_deviation)
        r = np.random.normal(self.r_average, self.r_standard_deviation)
        return [delta, r]


class Sample:
    def __init__(self, x, y, theta, direction):
        self.x = x
        self.y = y
        self.theta = theta
        self.direction = direction

    def __str__(self):
        return "(x: {}, y: {}, theta: {}, direction: {})".format(self.x, self.y, self.theta, self.direction)

    def __eq__(self, other):
        return self.x == other.x and \
               self.y == other.y and \
               self.theta == other.theta and \
               self.direction == other.direction

    def __hash__(self):
        return str(self)

    def get_point_in_geometry(self):
        return Point(self.x, self.y)

    def perform_action(self, action_type: int, action: Action):  # GenerateSampleTransition
        point = self.get_point_in_geometry()
        delta, r = action.sample()
        arc = Arc(point, self.theta, r, delta, self.direction)

        next_x = arc.point_b.x
        next_y = arc.point_b.y
        if action_type == 0:
            next_theta = (self.theta + delta / r) % (2 * math.pi)
        else:
            next_theta = (self.theta - delta / r) % (2 * math.pi)
        next_direction = action_type
        next_sample = Sample(next_x, next_y, next_theta, next_direction)
        return next_sample, arc


def convert_sample_from_kd_tree_to_problem(sample_in_kd_tree):
    return Sample(sample_in_kd_tree[0], sample_in_kd_tree[1], sample_in_kd_tree[2], int(sample_in_kd_tree[4] / 10000000000))


def convert_sample_from_problem_to_kd_tree(sample_in_problem: Sample):
    return [sample_in_problem.x, sample_in_problem.y, sample_in_problem.theta,
            sample_in_problem.theta, sample_in_problem.direction * 10000000000]


def distance_for_sample(sample_a: Sample, sample_b: Sample) -> float:
    return math.sqrt((sample_a.x - sample_b.x) ** 2
                     + (sample_a.y - sample_b.y) ** 2
                     + 2.0 * (sample_a.theta - sample_b.theta) ** 2) \
           + (sample_a.direction != sample_b.direction) * math.inf


class Solution:
    def __init__(self):
        self.samples = []
        self.actions = []
        self.trajectories = []
        self.successful = True


class Problem:
    def __init__(self, obstacles, start: Sample, goal: Sample, t: float, number_of_nodes,
                 number_of_sample_per_transition, x_limit, y_limit,
                 actions: [Action]):
        self.obstacles = []  # list of Polygon: [Polygon]
        for obstacle in obstacles:
            self.obstacles.append(Polygon(obstacle))
        self.start = start
        self.goal = goal
        self.t = t  # range of goal
        self.number_of_nodes = number_of_nodes  # n: number of nodes to place in the roadmap
        self.number_of_sample_per_transition = number_of_sample_per_transition
        # m: number of sample points to generate for each transition

        self.x_limit = x_limit  # limit of space on x-axis [0,x-limit]
        self.y_limit = y_limit  # limit of space on y-axis [0,x-limit]

        self.solution = Solution()
        self.solution.samples.append(start)
        self.actions = actions

        self.nodes = self.sampling()
        # self.nodes.append(start)
        # self.nodes.append(goal)
        nodes_for_kd_tree = []
        for node in self.nodes:
            nodes_for_kd_tree.append([node.x, node.y, node.theta, node.theta, node.direction * 10000000000])
            # trick here
        self.kd_tree = KDTree(np.array(nodes_for_kd_tree))
        self.obstacle_node = Sample(math.inf, math.inf, 0, 0)
        self.nodes.append(self.obstacle_node)  # obstacle state

    def clear(self):  # save info for next planning, but delete the solution
        self.solution = Solution()
        new_coord = np.random.rand(2)
        new_coord[0] *= 2 * math.pi
        new_coord[1] = math.floor(2 * new_coord[1])
        current_sample = Sample(self.start.x, self.start.y, new_coord[0], new_coord[1])
        self.solution.samples.append(current_sample)

    def in_goal_range(self, node: Sample) -> bool:
        # just use this function, do not care about the name
        return distance_for_collision_checking(self.goal.get_point_in_geometry(),
                                               node.get_point_in_geometry()) <= self.t

    def check_collision_for_sample(self, sample: Sample):
        if sample.x >= self.x_limit or sample.y >= self.y_limit:
            return True
        for obstacle in self.obstacles:
            if obstacle.contains(sample.get_point_in_geometry()):
                return True
        return False

    def check_collision_for_trajectory(self, trajectory: Arc):
        s0 = Segment(Point(0, 0), Point(0, self.y_limit))
        if s0.has_intersect_point_with_arc(trajectory):
            return True
        s1 = Segment(Point(0, self.y_limit), Point(self.x_limit, self.y_limit))
        if s1.has_intersect_point_with_arc(trajectory):
            return True
        s2 = Segment(Point(self.x_limit, self.y_limit), Point(self.x_limit, 0))
        if s2.has_intersect_point_with_arc(trajectory):
            return True
        s3 = Segment(Point(self.x_limit, 0), Point(0, 0))
        if s3.has_intersect_point_with_arc(trajectory):
            return True
        for obstacle in self.obstacles:
            if obstacle.intersect_with_arc(trajectory):
                return True
        return False

    def sampling(self):
        nodes = []
        while len(nodes) < self.number_of_nodes:
            new_coord = np.random.rand(4)
            new_coord[0] *= self.x_limit
            new_coord[1] *= self.y_limit
            new_coord[2] *= 2 * math.pi
            new_coord[3] = math.floor(2 * new_coord[3])
            sample = Sample(new_coord[0], new_coord[1], new_coord[2], new_coord[3])
            if self.check_collision_for_sample(sample):
                nodes.append(sample)
        return nodes

    def get_transitions(self, node: Sample, action_type: int, action: Action):
        map_next_node_probability = dict()
        for i in range(self.number_of_sample_per_transition):
            next_sample, trajectory = node.perform_action(action_type, action)
            if self.check_collision_for_sample(next_sample) or self.check_collision_for_trajectory(trajectory):
                nearest_sample = self.nodes[-1]
            else:
                nearest_sample = convert_sample_from_kd_tree_to_problem(
                    self.nodes[self.kd_tree.query(
                        [convert_sample_from_problem_to_kd_tree(next_sample)]
                    )[1]]
                )  # get from kd-tree
            if nearest_sample in map_next_node_probability:
                map_next_node_probability[nearest_sample] += 1 / self.number_of_sample_per_transition
            else:
                map_next_node_probability[nearest_sample] = 1 / self.number_of_sample_per_transition
        return map_next_node_probability

    def build_SMR(self):
        edges = [{} for i in range(len(self.actions))]
        for node in self.nodes:
            for i in range(len(self.actions)):
                transitions = self.get_transitions(node, i, self.actions[i])
                for next_node, probability in transitions.items():
                    if len(edges[i][node]) == 0:
                        edges[i][node] = [(next_node, probability)]
                    else:
                        edges[i][node].append((next_node, probability))
                    #edges[i][node].append((node, next_node, probability))
        return [self.nodes, edges]

    def run_MDP(self, nodes, edges):
        u1 = dict([(s, 0) for s in nodes])
        action_result = {}
        for s in nodes:
            if self.in_goal_range(s):
                u1[s] = 1
        delta, gamma, epsilon = 1, 0.001, 0.001
        while delta > epsilon:
            u = u1.copy()
            delta = 0
            for s in nodes:
                max_val = -1
                for a in range(len(self.actions)):
                    tmp_val = 0
                    if len(edges[a][s]) > 0:
                        for (t, p) in edges[a][s]:
                            tmp_val += p * (-gamma + u[t])
                    if tmp_val > max_val:
                        max_val = tmp_val
                        action_result[s] = self.actions[a]
                u1[s] = max_val
                delta = max(delta, abs(u1[s] - u[s]))
            return action_result

    def solve(self):
        nodes, edges = self.build_SMR()
        map_node_action = self.run_MDP(nodes, edges)
        while not self.in_goal_range(self.solution.samples[-1]):
            cur_sample = self.solution.samples[-1]
            nearest_sample = convert_sample_from_kd_tree_to_problem(
                self.nodes[self.kd_tree.query(
                    [convert_sample_from_problem_to_kd_tree(cur_sample)]
                )[1]]
            )

            best_action = map_node_action[nearest_sample]
            next_sample, trajectory = cur_sample.perform_action(best_action)
            self.solution.samples.append(next_sample)
            self.solution.actions.append(best_action)
            self.solution.trajectories.append(trajectory)
            if self.check_collision_for_sample(next_sample) or self.check_collision_for_trajectory(trajectory):
                self.solution.successful = False
                break
        return self.solution
