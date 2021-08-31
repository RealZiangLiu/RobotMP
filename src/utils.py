#!/usr/bin/env python
import time

import pybullet as p
import numpy as np

num_joints = 7

def set_joint_positions(body, joint_positions):
    for i in range(7):
        p.resetJointState(body, i, targetValue=joint_positions[i], targetVelocity=0)


def get_joint_positions(body):
    return [p.getJointState(body, i)[0] for i in range(p.getNumJoints(body))]


def in_collision(body, joint_positions):
    # prev_joint_positions = [p.getJointState(body, i)[0] for i in range(p.getNumJoints(body))]
    set_joint_positions(body, joint_positions)
    p.performCollisionDetection()
    is_colliding = False
    if len(p.getContactPoints(body)) > 0:
        is_colliding = True
    # print(len(p.getContactPoints(body)))
    # set_joint_positions(body, prev_joint_positions)
    # if is_colliding:
    #     print("in collision")
    # else:
    #     print("not in collision")
    return is_colliding


def compute_inverse_kinematics(body, ee_index, position, orientation,
                               lower_limits=[-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
                               upper_limits=[2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973],
                               max_iter=20, epsilon=1e-4):
    return p.calculateInverseKinematics(body, ee_index, position, orientation,
                                        lowerLimits=lower_limits,
                                        upperLimits=upper_limits,
                                        maxNumIterations=max_iter,
                                        residualThreshold=epsilon)


def move_to_joint_positions(body, joint_positions, position_gain=0.05, interval=0.05, max_iter=1000, epsilon=1e-4):
    found_collision = False

    for i in range(len(joint_positions)):
        p.setJointMotorControl2(body, i, controlMode=p.POSITION_CONTROL, targetPosition=joint_positions[i],
                                targetVelocity=0,
                                positionGain=position_gain)
    for i in range(max_iter):
        p.stepSimulation()
        # collision check
        if len(p.getContactPoints(body)) > 0:
            found_collision = True
            break
        if np.linalg.norm(np.array(joint_positions) - np.array([p.getJointState(body, i)[0] for i in range(len(joint_positions))])) < epsilon:
            break
        time.sleep(interval)

    return found_collision
