#!/usr/bin/env python

import sys
import os
import math
import time
# set OMPL install path or add path to system path directly
sys.path.insert(0, os.path.join('/home/ziang/Workspace/lib', 'ompl-1.5.2/py-bindings'))

sys.path.insert(0, os.path.join('/home/ziang/Workspace/Github/RobotMP', 'pybullet-planning'))

import numpy as np
from numpy.linalg import norm
import pybullet as p

from pybullet_tools.pr2_utils import TOP_HOLDING_LEFT_ARM, PR2_URDF, DRAKE_PR2_URDF, \
    SIDE_HOLDING_LEFT_ARM, PR2_GROUPS, open_arm, get_disabled_collisions, REST_LEFT_ARM, rightarm_from_leftarm
from pybullet_tools.utils import set_base_values, joint_from_name, quat_from_euler, set_joint_position, \
    set_joint_positions, add_data_path, connect, plan_base_motion, plan_joint_motion, enable_gravity, \
    joint_controller, dump_body, load_model, joints_from_names, wait_if_gui, disconnect, get_joint_positions, \
    get_link_pose, link_from_name, HideOutput, get_pose, wait_if_gui, load_pybullet, set_quat, Euler, PI, RED, add_line, \
    wait_for_duration, LockRenderer, base_aligned_z, Point, set_point, get_aabb, stable_z_on_aabb, AABB, get_joint_name, \
    get_movable_joints, get_collision_fn

import robotmp as rmp
from src.utils import in_collision, compute_inverse_kinematics, set_joint_positions

BENCHMARK_ITERS = 1000
DO_BENCHMARK = True

# setup bullet physics engine
clid = p.connect(p.SHARED_MEMORY)
if clid < 0:
  p.connect(p.GUI)
  print("[Info] Connected to PyBullet GUI mode.")

panda = p.loadURDF("PandaRobot.jl/deps/Panda/panda.urdf", basePosition=[0., 0., 0.], useFixedBase=True)
arm1 = p.loadURDF("PandaRobot.jl/deps/Panda/panda.urdf", basePosition=[0.3, 0.3, 0.], useFixedBase=True)

ee_index = 8
lower_limits = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
upper_limits = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]

""" 
  customize state validity checker function
  Input: current state
  Output: True if current state is valid (collision free, etc.)
"""
def is_safe(state):
    return not in_collision(panda, state)
    # return True

def print_stats(method, costs, times, states):
    print("===============================")
    print("[INFO]: Stats for", method)
    print("[INFO]: Num iters: ", BENCHMARK_ITERS)
    print("[INFO]: Average path cost: ", sum(costs)/len(costs))
    print("[INFO]: Average time: ", sum(times)/len(times))
    print("[INFO]: Average waypoints: ", sum(states)/len(states))

def run_ompl_planner():
    target_pos = [0.058, 0.45, 0.73]
    orn = p.getQuaternionFromEuler([0, -math.pi, 0])

    # compute IK
    target_joint_pos = compute_inverse_kinematics(panda, ee_index, target_pos, orn)[:7]
    joints = get_movable_joints(panda)

    collision_fn = get_collision_fn(panda, joints, obstacles=[arm1], attachments=[], self_collisions=True, disabled_collisions=set(),
                                    custom_limits={}, max_distance=0.,
                                    use_aabb=False, cache=True)
    # create planner
    planner = rmp.OMPLPlanner(state_space_bounds=[lower_limits, upper_limits],
                              state_validity_checker=is_safe,
                              planner="est")

    start_state = [0., 0., 0., -1.5, 0., 1.5, 0.]
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
    if not DO_BENCHMARK:
        path, cost, t = planner.plan(time_limit=10.0)
        if path is not None:
            print(dir(path))
            print(path.getStateCount())
            for idx in range(path.getStateCount()):
                set_joint_positions(panda, path.getState(idx))
                time.sleep(5.0/path.getStateCount())
            # input("Enter to quit...")
    else:
        costs = []
        times = []
        states = []

        for iter in range(BENCHMARK_ITERS):
            path, cost, t = planner.plan(time_limit=10.0, VERBOSE=False)
            costs.append(cost)
            times.append(t)
            states.append(path.getStateCount())
            print("[INFO] Iteration:", iter, "cost:", cost, "time:", t)

        print_stats("OMPL", costs, times, states)


def run_bullet_planner():
    target_pos = [0.058, 0.45, 0.73]
    orn = p.getQuaternionFromEuler([0, -math.pi, 0])

    # compute IK
    target_joint_pos = compute_inverse_kinematics(panda, 8, target_pos, orn)[:7]
    print(target_joint_pos)
    joints = get_movable_joints(panda)
    print('Joints', [get_joint_name(panda, joint) for joint in joints])

    start_state = [0., 0., 0., -1.5, 0., 1.5, 0., 0., 0.]
    set_joint_positions(panda, start_state)

    if not DO_BENCHMARK:
        t_1 = time.perf_counter()
        arm_path = plan_joint_motion(panda, list(range(7)), target_joint_pos, obstacles=[arm1], algorithm='rrt_connect')
        t_2 = time.perf_counter()
        print(f"[Info] Time: {t_2 - t_1:0.4f}s")

        if arm_path is not None:
            print()
            print("[INFO] Found Solution!")
            print("[INFO] Solution size: ", len(arm_path))
            print("[INFO] Path: ")
            for state in arm_path:
                print(state)

            cost = 0.0
            for i in range(1, len(arm_path)):
                cost += norm(np.array(arm_path[i]) - np.array(arm_path[i-1]))
            print("[INFO] Solution cost: ", cost)

            # visualize
            for i in range(len(arm_path)):
                set_joint_positions(panda, arm_path[i])
                time.sleep(5.0/len(arm_path))
    else:
        costs = []
        times = []
        states = []
        for iter in range(BENCHMARK_ITERS):
            set_joint_positions(panda, start_state)
            t_1 = time.perf_counter()
            arm_path = plan_joint_motion(panda, list(range(7)), target_joint_pos, obstacles=[arm1], algorithm='birrt')
            t_2 = time.perf_counter()

            cost = 0.0
            for i in range(1, len(arm_path)):
                cost += norm(np.array(arm_path[i]) - np.array(arm_path[i-1]))
            costs.append(cost)
            times.append(t_2 - t_1)
            states.append(len(arm_path))

        print_stats("PyBullet Planning", costs, times, states)



def main():

    # run_bullet_planner()
    run_ompl_planner()
    p.disconnect()


if __name__ == "__main__":
    main()
