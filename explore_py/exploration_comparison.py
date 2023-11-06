import numpy as np
import argparse
import matplotlib.pyplot as plt
from cprint import cprint

from explore_test import ExploreTest
from exploration import ExplorationPlanner, FrontierPlanner, MIPlanner

def score_exploration(tester):
    frontier_faster_than_random = []
    mi_faster_than_random = []

    mi_faster_than_frontier = []
    for env in envs:
        cprint.info("Exploring %s using Random exploration" % (env))
        E1 = None
        try:
            E1 = tester.run(scenario=env, planner=ExplorationPlanner)
        except RuntimeError or NotImplementedError:
            cprint.err("Exploring %s using Random exploration failed." %
                       (env), interrupt=False)

        tester.reset()

        cprint.info("Exploring %s using Frontier-based exploration" % (env))
        E2 = None
        try:
            E2 = tester.run(scenario=env, planner=FrontierPlanner)
        except RuntimeError or NotImplementedError:
            cprint.err(
                "Exploring %s using Frontier-based exploration failed." % (env), interrupt=False)

        tester.reset()

        cprint.info("Exploring %s using MI-based exploration" % (env))
        E3 = None
        try:
            E3 = tester.run(scenario=env, planner=MIPlanner)
        except RuntimeError or NotImplementedError:
            cprint.err(
                "Exploring %s using Frontier-based exploration failed." % (env), interrupt=False)

        tester.reset()

        fig, ax = plt.subplots()
        ax.set_title('Entropy reduction for %s environment' % (env))
        ax.set_xlabel('Time Step')
        ax.set_ylabel('Map Entropy in bits')

        if E1 is not None and E2 is not None:
            if np.mean(E2 - E1) < 0:
                frontier_faster_than_random.append(True)
            else:
                frontier_faster_than_random.append(False)
            ax.plot(E1, label='Random')
            ax.plot(E2, label='Frontier')

        if E3 is not None and E1 is not None:
            if np.mean(E3 - E1) < 0:
                mi_faster_than_random.append(True)
            else:
                mi_faster_than_random.append(False)
            ax.plot(E3, label='MI')

        if E3 is not None and E2 is not None:
            if np.mean(E3 - E2) < 0:
                mi_faster_than_frontier.append(True)
            else:
                mi_faster_than_frontier.append(False)

        ax.legend()

    print(frontier_faster_than_random)
    print(mi_faster_than_random)
    print(mi_faster_than_frontier)

    score_3_1 = 0.0
    score_4_2 = 0.0

    # For Tasks 3.1, 2.2, and 2.1
    if len(frontier_faster_than_random) == 4:
        if sum(frontier_faster_than_random) >= 3:
            cprint.ok("[Tasks 3.1 and 2.2]: Full Credit.")
            score_3_1 = 1.0
        else:
            cprint.warn("[Tasks 3.1 and 2.2]: Frontier-based planner must outperform random planner"
                         " in at least one environment. Please check your implementation.")
    else:
        cprint.err("[Tasks 3.1 and 2.2]: Frontier-based, random, or both exploration"
                    " methods are not correctly implemented.", interrupt=False)

    # For Task 4.2
    if len(mi_faster_than_random) == 4:
        if sum(mi_faster_than_random) >= 3 and sum(mi_faster_than_frontier) >= 2:
            cprint.ok("[Task 4.2]: Full Credit.")
            score_4_2 = 1.0
        else:
            cprint.ok("[Task 4.2]: Partial Credit.")
            score_4_2 = 0.2 * (sum(mi_faster_than_frontier) + sum(mi_faster_than_random))
    else:
        cprint.err("[Task 4.2]: Frontier-based, random, or both exploration"
                    " methods are not correctly implemented.", interrupt=False)

    return score_3_1, score_4_2

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-max_time",
                        help="maximum duration of exploration",
                        type=int,
                        default=200)
    args = parser.parse_args()

    envs = ['simple-obstacle', 'office', 'maze-like', 'charrow-map']

    tester = ExploreTest(args.max_time, vis_on=False)

    scores = score_exploration(tester)
    print(scores)
    plt.show()