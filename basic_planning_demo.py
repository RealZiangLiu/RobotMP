#!/usr/bin/env python

import sys
import os
# set OMPL install path or add path to system path directly
sys.path.insert(0, os.path.join('/home/ziang/Workspace/lib', 'ompl-1.5.2/py-bindings'))

import numpy as np
from numpy.linalg import norm

import robotmp as rmp

""" 
  customize state validity checker function
  Input: current state
  Output: True if current state is valid (collision free, etc.)
"""
def is_safe(state):
  s = [state[i] for i in range(3)]
  return norm(s) < 5.0 and state[1] < 1.1

def main():
  # create planner
  planner = rmp.OMPLPlanner(state_space_bounds = [[-1., -1., -1.],[1., 1., 1.]],
                            state_validity_checker = is_safe,
                            planner = "rrt_connect")
  
  # set start and goal states
  planner.set_start_and_goal(start = [-0.5, 1.0, 1.0], goal = [0.5, 0.5, 0.5])
  
  # solve!
  path, cost, t = planner.plan(time_limit = 3.0)

main()