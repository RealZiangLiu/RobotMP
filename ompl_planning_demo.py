
#!/usr/bin/python3


from os.path import abspath, dirname, join
import sys
sys.path.insert(0, join('/home', 'iliad', 'ompl', 'py-bindings'))
# print(sys.path)
from ompl import base as ob
from ompl import geometric as goem_planners

import time

def isStateValid(state):
    return state[0] < .9

def plan():
    # create astate space
    space = ob.RealVectorStateSpace(7)
    # set state limits
    bounds = ob.RealVectorBounds(7)
    bounds_lower = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
    bounds_upper = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]

    for i in range(7):
      bounds.setLow(i, bounds_lower[i])
      bounds.setHigh(i, bounds_upper[i])
    space.setBounds(bounds)

    # construct an instance of space information from this state space
    si = ob.SpaceInformation(space)
    # set state validity checking for this space
    si.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
    # create a random start state
    start = ob.State(space)
    start.random()
    start[0] = 0.1
    # create a random goal state
    goal = ob.State(space)
    goal.random()
    # create a problem instance
    pdef = ob.ProblemDefinition(si)
    # set the start and goal states
    pdef.setStartAndGoalStates(start, goal)
    # create a planner for the defined space
    planner = goem_planners.RRTstar(si)
    # set the problem we are trying to solve for the planner
    planner.setProblemDefinition(pdef)
    # perform setup steps for the planner
    planner.setup()
    # print the settings for this space
    print(si.settings())
    # print the problem settings
    print(pdef)
    # attempt to solve the problem within one second of planning time
    t_1 = time.perf_counter()
    solved = planner.solve(2.0)
    t_2 = time.perf_counter()

    if solved:
        # get the goal representation from the problem definition (not the same as the goal state)
        # and inquire about the found path
        path = pdef.getSolutionPath()
        print("Found solution:\n%s" % path)
        print(f"Time: {t_2 - t_1:0.4f}s")
        print("Solution Cost: %f\n" %path.length())
    else:
        print("No solution found")



if __name__ == "__main__":
    print("")
    plan()
