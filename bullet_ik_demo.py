#!/usr/bin/env python
import time
import math

import pybullet as p
import pybullet_data
import numpy as np

from src.utils import in_collision, compute_inverse_kinematics, move_to_joint_positions, set_joint_positions

clid = p.connect(p.SHARED_MEMORY)

if (clid < 0):
  p.connect(p.GUI)

p.setPhysicsEngineParameter(enableConeFriction=0)
# p.setRealTimeSimulation(True)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

panda = p.loadURDF("PandaRobot.jl/deps/Panda/panda.urdf", basePosition=[0., 0., 0.], useFixedBase=True)
fake = p.loadURDF("PandaRobot.jl/deps/Panda/panda.urdf", basePosition=[0.2, 0.2, 0.], useFixedBase=True)

for i in range(p.getNumJoints(panda)):
    print(p.getJointInfo(panda, i))

pandaEndEffectorIndex = 8
jointLowerLimits = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
jointUpperLimits = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
targetPos = [0.058, 0.35, 0.8259999999999999]
orn = p.getQuaternionFromEuler([0, -math.pi, 0])

print("initial position", p.getLinkState(panda, pandaEndEffectorIndex)[0])

jointPositions = compute_inverse_kinematics(panda, pandaEndEffectorIndex, targetPos, orn, jointLowerLimits, jointUpperLimits, max_iter=10000, epsilon=1e-5)
print(jointPositions)

collision = move_to_joint_positions(panda, jointPositions)

print("end position", p.getLinkState(panda, pandaEndEffectorIndex)[0])
print("Collision: ", collision)
input("Enter to quit...")
p.disconnect()
