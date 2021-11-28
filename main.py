import copy
import math

import matplotlib

import smr
import numpy
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

if __name__ == '__main__':
    print("Number of nodes?")  # Number of states
    number_of_nodes = int(input())
    print("Number of sample points to generate for each transition?")
    number_of_samples_to_gen_per_transition = int(input())
    print("Number of tests?")
    number_of_tests = int(input())

    obs_list = [[(0.0, 4.0), (3.0, 4.0), (3.0, 6.0), (0.0, 6.0)], [(4.0, 4.0), (4.0, 6.0), (5.0, 6.0), (5.0, 4.0)]]
    st_sample = smr.Sample(2.0, 2.0, 0, 0)
    ed_sample = smr.Sample(2.0, 8.0, 0, 0)
    actions = [
        smr.Action(0.5, 2.5, 0.1, 0.5),
        smr.Action(0.5, 2.5, 0.2, 1.0)
    ]

    problem = smr.Problem(obs_list, st_sample, ed_sample, 1,
                          number_of_nodes, number_of_samples_to_gen_per_transition, 10, 10, actions)

    successful_times = 0
    for _ in range(number_of_tests):
        problem.clear()
        solution = problem.solve()
        trajectories = solution.trajectories

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

        for trajectory in trajectories:
            plot_arc = patches.Arc((trajectory.center.x, trajectory.center.x), trajectory.r, trajectory.r, 0,
                                   trajectory.degree_a * 360 / 2 * math.pi, trajectory.degree_b * 360 / 2 * math.pi)

        if solution.success:
            successful_times += 1
    # TODO: record probability of success
