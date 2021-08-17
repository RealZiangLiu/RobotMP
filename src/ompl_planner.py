#!/usr/bin/env python

import sys
import os
import time

from ompl import base as ob
from ompl import geometric as gp

# sys.path.insert(0, os.path.join('/home', 'ziang', 'Workspace', 'lib', 'ompl-1.5.2', 'py-bindings'))

class OMPLPlanner:
  """
    Wrapper class for OMPL planners

    Abstracts away complicated OMPL API not needed for simple robotics planner task
  """
  
  planner_map = {
    "rrt_star": gp.RRTstar,
    "rrt": gp.RRT,
    "rrt_connect": gp.RRTConnect,
    "lazy_rrt": gp.LazyRRT,
    "prm": gp.PRM,
    "prm_star": gp.PRMstar,
    "lazy_prm": gp.LazyPRM,
    "lazy_prm_star": gp.LazyPRMstar,
    "est": gp.EST,
    "fmt": gp.FMT,
    # and more...
  }

  def __init__(self,
               state_space_bounds=[[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                   [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]],
               state_validity_checker=lambda state : True,
               planner="rrt_connect"
              ):
    assert len(state_space_bounds) == 2, "<state_space_bounds> must have lengh 2."
    assert len(state_space_bounds[0]) == len(state_space_bounds[1]), "<state_space_bounds> and <state_space_dimension> size mismatch."
    
    self.state_space_dimension = len(state_space_bounds[0])
    self.state_space_bounds = state_space_bounds
    self.state_validity_checker = state_validity_checker
    self.planner_str = planner

    self.state_space = None
    self.state_bounds = None
    self.space_information = None
    self.start_state = None
    self.goal_state = None
    self.problem = None
    self.planner = None

    self.setup()

  def setup(self):
    # set up state space and bounds
    self.state_space = ob.RealVectorStateSpace(self.state_space_dimension)
    self.state_bounds = ob.RealVectorBounds(self.state_space_dimension)
    for i in range(self.state_space_dimension):
      self.state_bounds.setLow(i, self.state_space_bounds[0][i])
      self.state_bounds.setHigh(i, self.state_space_bounds[1][i])
    self.state_space.setBounds(self.state_bounds)

    # construct an instance of space information from this state space
    self.space_information = ob.SpaceInformation(self.state_space)
    # set state validity checking for this space
    self.space_information.setStateValidityChecker(ob.StateValidityCheckerFn(self.state_validity_checker))
    # create a problem instance
    self.problem = ob.ProblemDefinition(self.space_information)
    # set start and goal states to random
    self.start_state = ob.State(self.state_space)
    self.goal_state = ob.State(self.state_space)
    self.start_state.random()
    self.goal_state.random()
    # set the start and goal states
    self.problem.setStartAndGoalStates(self.start_state, self.goal_state)
    # setup planner
    self.planner = OMPLPlanner.planner_map[self.planner_str](self.space_information)
    # set the problem we are trying to solve for the planner
    self.planner.setProblemDefinition(self.problem)
    # perform setup steps for the planner
    self.planner.setup()
    # print the settings for this space
    # print(self.space_information.settings())
    # print the problem settings
    # print(self.problem)
  
  def set_start_and_goal(self, start, goal):
    assert len(start) == self.state_space_dimension and len(goal) == self.state_space_dimension, "Incorrect <start>/<goal> dimension."
    for i in range(self.state_space_dimension):
      self.start_state[i] = start[i]
      self.goal_state[i] = goal[i]
    self.problem.setStartAndGoalStates(self.start_state, self.goal_state)
    print(f"[Info] Successfully set start: {start} goal: {goal}")
  
  def set_step_size(self, step_size):
    self.planner.setRange(step_size)

  def plan(self, time_limit=2.0):
    
    t_1 = time.perf_counter()
    solved = self.planner.solve(time_limit)
    t_2 = time.perf_counter()
    if solved:
      path = self.problem.getSolutionPath()
      print("[Info] Found solution:\n%s" % path)
      print(f"[Info] Time: {t_2 - t_1:0.4f}s")
      print("[Info] Solution Cost: %f\n" %path.length())
      return path, path.length(), t_2 - t_1
    else:
      print("[Info] Solver failed.")
      return None, None, t_2 - t_1
    
if __name__ == "__main__":
  op = OMPLPlanner(planner="rrt_connect")
  op.set_start_and_goal([0,0,0,0,0,0,0], [1,1,1,1,1,1,0.5])
  op.set_step_size(0.5)
  path, cost, t = op.plan(time_limit=2.0)
