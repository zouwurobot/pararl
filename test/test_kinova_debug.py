import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import time
import pybullet as p

import random
import pybullet_data
from pkg_resources import parse_version
import os
from deepx import *
import cv2


from otter.gym.bullet import kinova
_urdfRoot = pybullet_data.getDataPath()
_robot_urdfRoot = '../otter/gym/bullet/assets/urdf'

robot_init_pos = [ -7.81, 3.546, 12.883, 0.833, -2.753, 4.319, 5.917 ,1, 1, 1]


p.connect(p.GUI)
p.resetSimulation()
p.setPhysicsEngineParameter(numSolverIterations=150)
p.setTimeStep(0.01)

p.loadURDF(os.path.join(_urdfRoot, "plane.urdf"), [0, 0, -1])
tableUid = p.loadURDF(os.path.join( _urdfRoot, "table/table.urdf"),
                           [0.0000000, -0.50000, -.620000],
                           [0.000000, 0.000000, 0.0, 1.0])

cubeUid = p.loadURDF(os.path.join( _robot_urdfRoot, "Cube.urdf"),
                          [0.0, -0.55, 0.15],
                          [0, 0, 0, 1],
                          useFixedBase=True)
p.setGravity(0, 0, -9.81)

# camera configuration
xpos = 0
ypos = -0.55 - 0.12 * random.random()
ang = math.pi / 2
zpos = 0.25
orn = p.getQuaternionFromEuler([ang, 0, 0])
CameraViewMatrix = p.computeViewMatrixFromYawPitchRoll([xpos, ypos, zpos], 1, 180, -60, 0, 2)

sCameraProjMatrix = p.computeProjectionMatrixFOV(80, 0.5, 0, 8)


kinova = kinova.Kinova(p,
                            robot_type='j2s7s300',
                            urdfRootPath=_robot_urdfRoot,
                            timeStep=0.01,
                            building_env=False,  # use gym env
                            useInverseKinematics=True,  # IMPORTANCE! It determines the mode of the motion.
                            torque_control_enabled=False,
                            is_fixed=True,
                            init_position = robot_init_pos)


while 1:
    # x = np.random.uniform(-1, 1,)* 0.01
    # y = np.random.uniform(-1, 1,)* 0.01
    # z = np.random.uniform(-1, 1,)* 0.01
    # action = [x, y, z,   0.708, -0.019, 0.037, 0.705,   0.3]
    #
    # action = [0.20, -0.2, 0.35, 0.708, -0.019, 0.037, 0.705, 0.3]
    # kinova.ApplyAction_abs(action)
    kinova.DebugJointControl()
    for i in range(10):

        p.stepSimulation()

    time.sleep(0.1)

