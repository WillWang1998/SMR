import numpy as np


class ConfSpace:
    def __init__(self, obstacle, start, goal, needle_length, node_cnt, sample_per_transition, x_limit, y_limit,
                 omega_limit):
        # TODO: add transition cost for MDP
        self.obstacle = obstacle  # list of list: obstacle[vertices]
        self.node = {}  # map: coordinate in c-space -> node
        self.start = start
        self.goal = goal
        self.needle_length = needle_length  # Dubins car length
        self.node_cnt = node_cnt  # n: number of nodes to place in the roadmap
        self.sample_per_transition = sample_per_transition  # m: number of sample points to generate for each transition
        self.x_limit = x_limit  # limit of space on x-axis [0,x-limit]
        self.y_limit = y_limit  # limit of space on y-axis [0,x-limit]
        self.omega_limit = omega_limit  # limit of angular velocity [-omega-limit, +omega-limit]


    def build_smr(self, sample_cnt):
        while len(self.node) < self.node_cnt:
            new_coord = np.random.rand(3)
            new_coord[0] *= self.x_limit
            new_coord[1] *= self.y_limit
            new_coord[2] *= self.omega_limit * 2 - self.omega_limit
            coord_tuple = tuple(i for i in new_coord) # TODO: check whether point in obstacle

            self.node[coord_tuple] = Node(coord_tuple)

        for _, node in self.node:
            node.prob_dist = self.get_transition(node) # V = self.node; s = node; m = self.sample_per_transition


    def get_transition(self, node):
        # TODO: fill it
        return {}




class Node:
    def __init__(self, conf):
        self.conf = conf  # coordinate in C space (0, 0, 0)
        self.prob_dist = {}  # probability distribution; map: neighbor node -> probability reach that node
        self.value = 0.0
        self.reward = 0.0  # goal is 1, obstacle is -1


