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
                            init_configuration   = robot_init_pos)


pos = [-0.06947440654039383, -0.5439319014549255, 0.36741364002227783]
#orn = [0.708, -0.019, 0.037, 0.705]
orn = [0,0,0,1]

orn = p.getQuaternionFromEuler([3.0375982016633984, 0.09978265991422353, 0.09429909105253739])
for i in range(10):
    jointPoses = kinova._pybullet_client.calculateInverseKinematics(kinova.kinovaUid,kinova.EndEffectorIndex,pos,orn,
                                                                            lowerLimits=kinova.ll, upperLimits=kinova.ul, jointRanges=kinova.jr, restPoses=kinova.rp,
                                                                            residualThreshold=0.001, jointDamping=kinova.jd, )
    print(' Cal :', i, ' joint:' ,jointPoses)



# jointPoses = kinova._pybullet_client.calculateInverseKinematics(kinova.kinovaUid, kinova.EndEffectorIndex, pos, orn,
#                                                                     kinova.ll, kinova.ul, kinova.jr, kinova.rp,
#                                                                     jointDamping=kinova.jd, )



jointPoses = np.array([-7.024601836602551, 3.495153298757719, 12.117904125780235, 0.9354095495251066, -2.633787247015051, 4.220151040311783, 6.04334202071624])
current_joints = kinova.GetMotorAngles()
print('reset joint :', current_joints)
while True:
    pos = [-0.0647440654039383, -0.3439319014549255, 0.36741364002227783]

    jointPoses = kinova._pybullet_client.calculateInverseKinematics(kinova.kinovaUid, kinova.EndEffectorIndex, pos, orn,
                                                                    lowerLimits=kinova.ll, upperLimits=kinova.ul,
                                                                    jointRanges=kinova.jr, restPoses=kinova.rp,
                                                                    residualThreshold=0.001, jointDamping=kinova.jd, )

    for i in range(10):
        for i in range(1):
            motor_id = kinova.motorIndices[i]
            kinova._SetDesiredMotorAngleById(motor_id, jointPoses[i])

        p.stepSimulation()

   # obs = kinova.GetObservation()[:3]
    #print('pos: ', pos)
    #print('obs: ', obs)
    print('command joint :', jointPoses[0])
    current_joints = kinova.GetMotorAngles()[0]
    print('current joint :', current_joints)

    #print('delta = ',jointPoses - current_joints[:7] )

    print('-----------------------------------')

    time.sleep(0.1)

#
# while 1:
#     x = np.random.uniform(-1, 1,)* 0.01
#     y = np.random.uniform(-1, 1,)* 0.01
#     z = np.random.uniform(-1, 1,)* 0.01
#     action = [x, y, z,  0,0,0,1,   0.3]
#
#     #action = [0.20, -0.2, 0.35, 0.708, -0.019, 0.037, 0.705, 0.3]
#     kinova.ApplyAction(action)
#
#     for i in range(10):
#
#         p.stepSimulation()
#
#     time.sleep(0.1)

