import copy
import math
import os

import numpy as np

import smr
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

def expe1():
    number_of_nodes = 10000
    number_of_samples_to_gen_per_transition = 20
    number_of_tests = 1000
    obs_list = [[(0.0, 4.0), (1.5, 4.0), (2.0, 5.0), (1.5, 6.0), (0.0, 6.0)],
                [(3.5, 4.0), (5.0, 4.0), (5.0, 6.0), (3.5, 6.0), (4.0, 5.0)]]
    st_sample = smr.Sample(0.0, 2.0, 0, 0)
    ed_sample = smr.Sample(1.0, 8.0, 0, 0)
    actions = [
        smr.Action(0.8, 3.0, 0.08, 0.030), # average and standard deviation
        smr.Action(0.4, 3.0, 0.04, 0.030)
    ]

    problem = smr.Problem(obs_list, st_sample, ed_sample, 1,
                          number_of_nodes, number_of_samples_to_gen_per_transition, 8, 10, actions)

    successful_times = 0
    for _ in range(number_of_tests):
        solution = problem.solve()
        if solution.successful:
            successful_times += 1
    print("Success count: ", successful_times)

def expe2():
    number_of_nodes = [1000, 3000, 10000, 30000]
    number_of_samples_to_gen_per_transition = 20
    number_of_tests = 1000
    obs_list = [[(0.0, 4.0), (1.5, 4.0), (2.0, 5.0), (1.5, 6.0), (0.0, 6.0)],
                [(3.5, 4.0), (5.0, 4.0), (5.0, 6.0), (3.5, 6.0), (4.0, 5.0)]]
    st_sample = smr.Sample(0.0, 2.0, 0, 0)
    ed_sample = smr.Sample(1.0, 8.0, 0, 0)
    actions = [
        smr.Action(0.8, 3.0, 0.08, 0.030), # average and standard deviation
        smr.Action(0.4, 3.0, 0.04, 0.030)
    ]

    for j in number_of_nodes:
        problem = smr.Problem(obs_list, st_sample, ed_sample, 1,
                              j, number_of_samples_to_gen_per_transition, 8, 10, actions)

        successful_times = 0
        for _ in range(number_of_tests):
            solution = problem.solve()
            if solution.successful:
                successful_times += 1
        print("Success count: ", successful_times)

if __name__ == '__main__':
    print("Number of nodes?")  # Number of states
    number_of_nodes = int(input())
    print("Number of sample points to generate for each transition?")
    number_of_samples_to_gen_per_transition = int(input())
    print("Number of tests?")
    number_of_tests = int(input())

    obs_list = [[(0.0, 4.0), (1.5, 4.0), (2.0, 5.0), (1.5, 6.0), (0.0, 6.0)], [(3.5, 4.0), (5.0, 4.0), (5.0, 6.0), (3.5, 6.0), (4.0, 5.0)]]
    st_sample = smr.Sample(0.0, 2.0, 0, 0)
    ed_sample = smr.Sample(1.0, 8.0, 0, 0)
    actions = [
        smr.Action(0.8, 3.0, 0.01, 0.010),
        smr.Action(0.4, 3.0, 0.01, 0.010)
    ]

    problem = smr.Problem(obs_list, st_sample, ed_sample, 1,
                          number_of_nodes, number_of_samples_to_gen_per_transition, 8, 10, actions)

    successful_times = 0
    for _ in range(number_of_tests):
        solution = problem.solve()
        if solution.successful:
            successful_times += 1
        continue
        trajectories = solution.trajectories
        # for trajectory in trajectories:
        #     print("center {}".format(trajectory.center))
        #     print("form {} to {}".format(trajectory.point_a, trajectory.point_b))
        # for action in solution.actions:
        #     print("action: {}".format(action))
        # for sample in solution.samples:
        #     print("sample: {}".format(sample))

        fig = plt.figure()
        ax = fig.gca()
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)

        for obs in obs_list:
            temp = copy.deepcopy(obs)
            temp.append(temp[0])
            codes = [Path.MOVETO] + [Path.LINETO for i in range(len(temp) - 2)] + [Path.CLOSEPOLY]
            obs_path = Path(temp, codes)
            obs_patch = patches.PathPatch(obs_path)
            ax.add_patch(obs_patch)

        data = []
        plot_arc = patches.Arc((1, 8), 2, 2, 0,
                               0, 360)
        ax.add_patch(plot_arc)
        index = 0
        for trajectory in trajectories:
            # data.append([trajectory.point_a.x, trajectory.point_a.y])
            # plt.plot(trajectory.point_a.x, trajectory.point_a.y, 'ro')
            # print("(trajectory.center.x, trajectory.center.y), trajectory.r, trajectory.r, 0,"
            #       "trajectory.degree_a * 360 / (2 * math.pi), trajectory.degree_b * 360 / (2 * math.pi),",
            #       "trajectory.delta_theta",
            #       (trajectory.center.x, trajectory.center.y), trajectory.r, trajectory.r, 0,
            #       trajectory.degree_a * 360 / (2 * math.pi), trajectory.degree_b * 360 / (2 * math.pi),
            #       trajectory.delta_theta
            #       )
            degree_a = trajectory.degree_a * 360 / (2 * math.pi)
            degree_b = trajectory.degree_b * 360 / (2 * math.pi)
            if solution.actions[index] == 1:
                degree_a, degree_b = degree_b, degree_a
            plot_arc = patches.Arc((trajectory.center.x, trajectory.center.y), 2 * trajectory.r, 2 * trajectory.r, 0,
                                   degree_a, degree_b)
            ax.add_patch(plot_arc)
            index += 1

        # data = np.array(data)
        # ax.plot(data[:, 0], data[:, 1], '.-')
        file_path = os.path.dirname(os.path.abspath(__file__))
        plt.savefig(file_path + "/picture/{}.png".format(_))
        plt.clf()

        if solution.successful:
            successful_times += 1
            print("Test: ", _, "successful")
        else:
            print("Test: ", _, "failed")
    # TODO: record probability of success
    print("Success count: ", successful_times)
