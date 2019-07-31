import pybullet as p
import pybullet_data
import time
import os
import math

_urdfRoot  = pybullet_data.getDataPath()
_robot_urdfRoot = '../otter/gym/bullet/assets'

p.connect(p.GUI)
plane = p.loadURDF(os.path.join( _urdfRoot, "plane.urdf"), [0, 0, 0])
p.setGravity(0, 0, -9.8)
p.setTimeStep(1. / 500)


orn = p.getQuaternionFromEuler([math.pi/2, 0, 0])
cup_init_position = [0., -0.5, 0.3]

teacupUid = p.loadURDF(os.path.join(_robot_urdfRoot, "urdf/ball.urdf"), #urdf/coffee_cup.urdf  #coffee_cup.urdf
                                    cup_init_position,
                                    [orn[0], orn[1], orn[2], orn[3]],
                                    useFixedBase=False)


tea_dyn = p.getDynamicsInfo(teacupUid, -1)
print('tea:', tea_dyn)
#teacupUid = p.loadSDF(os.path.join(_robot_urdfRoot, "coke_can2/model.sdf") )
#p.resetBasePositionAndOrientation(teacupUid, [0, 0, 0.5], [0, 0, 0, 1])

p.setRealTimeSimulation(1)

while 1:
    pass