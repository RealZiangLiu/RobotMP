#!/usr/bin/env python

import sys
import os
import math
import time
# set OMPL install path or add path to system path directly
sys.path.insert(0, os.path.join('/home/ziang/Workspace/lib', 'ompl-1.5.2/py-bindings'))

import numpy as np
from numpy.linalg import norm
import pybullet as p

import robotmp as rmp
from src.utils import in_collision, compute_inverse_kinematics, set_joint_positions

# setup bullet physics engine
clid = p.connect(p.SHARED_MEMORY)
if clid < 0:
  p.connect(p.GUI)
  print("[Info] Connected to PyBullet GUI mode.")

panda = p.loadURDF("PandaRobot.jl/deps/Panda/panda.urdf", basePosition=[0., 0., 0.], useFixedBase=True)
obstacle = p.loadURDF("PandaRobot.jl/deps/Panda/panda.urdf", basePosition=[0.3, 0.3, 0.], useFixedBase=True)

ee_index = 8
lower_limits = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973, -10.0, -10.0]
upper_limits = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973, 10.0, 10.0]

""" 
  customize state validity checker function
  Input: current state
  Output: True if current state is valid (collision free, etc.)
"""
def is_safe(state):
    return not in_collision(panda, state)


def main():

    target_pos = [0.058, 0.45, 0.73]
    orn = p.getQuaternionFromEuler([0, -math.pi, 0])

    # compute IK
    target_joint_pos = compute_inverse_kinematics(panda, ee_index, target_pos, orn)

    # create planner
    planner = rmp.OMPLPlanner(state_space_bounds=[lower_limits, upper_limits],
                              state_validity_checker=is_safe,
                              planner="est")

    start_state = [0., 0., 0., -1.5, 0., 1.5, 0., 0., 0.]
    # set start and goal states
    print("start: ", start_state)
    set_joint_positions(panda, start_state)
    time.sleep(1.0)
    print("goal: ", target_joint_pos)
    set_joint_positions(panda, target_joint_pos)
    time.sleep(1.0)

    planner.set_start_and_goal(start=start_state, goal=target_joint_pos)
    planner.set_step_distance(0.5)

    # solve!
    path, cost, t = planner.plan(time_limit=10.0)
    if path is not None:
        print(dir(path))
        print(path.getStateCount())
        for idx in range(path.getStateCount()):
            set_joint_positions(panda, path.getState(idx))
            time.sleep(5.0/path.getStateCount())
        input("Enter to quit...")
    p.disconnect()


if __name__ == "__main__":
    main()
