import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print ("current_dir=" + currentdir)
os.sys.path.insert(0,currentdir)

import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import time
import pybullet as p
from . import kinova
import random
import pybullet_data
from pkg_resources import parse_version
import os
from deepx import *
import cv2

largeValObservation = 100

RENDER_HEIGHT = 720
RENDER_WIDTH = 960


from ..gym_wrapper import GymWrapper

__all__ = ['Reach']


class GymReach(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 50
    }


    def __init__(self, *args, **kwargs):
        self.__dict__.update(kwargs)
        self._isDiscrete = self.isDiscrete
        self._timeStep = 0.01
        self._urdfRoot = self.urdfRoot
        self._actionRepeat = self.actionRepeat
        self._isEnableSelfCollision = self.isEnableSelfCollision
        self._observation = []
        self._envStepCounter = 0
        self._renders = self._render
        self._maxSteps = self.maxSteps
        self.terminated = 0
        self._cam_dist = 1.3
        self._cam_yaw = 180
        self._cam_pitch = -40
        self._hard_reset = True
        self._robot_urdfRoot = os.path.join(currentdir, 'assets/urdf')

        GUI_FLAG = False
        self._p = p
        if self._renders:
            if not GUI_FLAG:
                cid = p.connect(p.SHARED_MEMORY)
                if (cid < 0):
                    cid = p.connect(p.GUI)
                    GUI_FLAG = True
                else:
                    p.connect(p.DIRECT)
                p.resetDebugVisualizerCamera(1.5, 180, -41, [0.32, 0.1, -0.00])
            else:
                p.connect(p.DIRECT)
        else:
            p.connect(p.DIRECT)

        # timinglog = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "kukaTimings.json")
        self._seed()
        self.reset()
        observationDim = len(self._get_obs())

        observation_high = np.array([largeValObservation] * observationDim)
        if (self._isDiscrete):
            self.action_space = spaces.Discrete(7)
        else:
            action_dim = 3
            self._action_bound = 1
            action_high = np.array([self._action_bound] * action_dim)
            self.action_space = spaces.Box(-action_high, action_high)
        self.observation_space = spaces.Box(-observation_high, observation_high)
        self.viewer = None

        self._hard_reset = self.hard_reset  # This assignment need to be after reset()

    def reset(self):
        self.robot_init_pos = self.get_reset_pos()
        self.goal_point = self.get_target_pos()
        if self._hard_reset:
            p.resetSimulation()
            p.setPhysicsEngineParameter(numSolverIterations=150)
            p.setTimeStep(self._timeStep)

            # build gym env
            p.loadURDF(os.path.join(self._urdfRoot, "plane.urdf"), [0, 0, -1])
            self.tableUid = p.loadURDF(os.path.join(self._urdfRoot, "table/table.urdf"),
                                       [0.0000000, -0.50000, -.620000],
                                       [0.000000, 0.000000, 0.0, 1.0])

            self.cubeUid = p.loadURDF(os.path.join(self._robot_urdfRoot, "Cube.urdf"),
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
            self._CameraViewMatrix = p.computeViewMatrixFromYawPitchRoll([xpos, ypos, zpos], 1, 180, -60, 0, 2)

            self._CameraProjMatrix = p.computeProjectionMatrixFOV(80, 0.5, 0, 8)


            self.kinova = kinova.Kinova(p,
                                        robot_type='j2s7s300',
                                        urdfRootPath=self._robot_urdfRoot,
                                        timeStep=self._timeStep,
                                        building_env=False,  # use gym env
                                        useInverseKinematics=True,  # IMPORTANCE! It determines the mode of the motion.
                                        torque_control_enabled=False,
                                        is_fixed=True,
                                        init_configuration = self.robot_init_pos)
        else:
            self.kinova.reset(reload_urdf=False)

        self.terminated = 0
        self._envStepCounter = 0
        p.stepSimulation()
        self._observation = self._get_obs()

        return np.array(self._observation)
    def get_reset_pos(self):
        if self.random_start:
            #TODO write a random function for start positon
            #start_pos = np.random.uniform(low=-np.pi, high=np.pi, size=self.model.nq)
            raise print('random function is not completed!')
        else:
            #start_pos = [math.pi/2, math.pi, math.pi, math.pi/6, 0, math.pi/2, 0, 1, 1, 1]
            #start_pos = [0, math.pi, 0, 0, 0, 0, 0, 1, 1, 1]
            start_pos = [-4.54, 3.438, 9.474, 0.749, 4.628, 4.472, 5.045, 1, 1, 1]
            start_pos = [-7.624, 2.814, 12.568, 0.758, -1.647, 4.492, 5.025, 1, 1, 1] # home position
            start_pos = [-7.81, 3.546, 12.883, 0.833, -2.753, 4.319, 5.917 ,1, 1, 1]  # init position

        return start_pos

    def get_target_pos(self):
        if self.random_target:
            goal = np.array([self.np_random.uniform(-0.2, 0.2), self.np_random.uniform(-0.2, 0.2)])
        else:
            goal = np.array(self.default_goal)
        return goal

    def __del__(self):
        p.disconnect()

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _get_obs(self):
        image_raw = p.getCameraImage(self.image_dim, self.image_dim,
                                     self._CameraViewMatrix, self._CameraProjMatrix)
        image_obs = image_raw[2][:,:,:3].flatten() # get the RGB data


        if self.is_image_state:
            self._observation = image_obs
        else:
            kinovaState = self.kinova.GetObservation()
            self._observation = \
                np.concatenate([np.array(kinovaState),
                                np.array(kinovaState[:3]) - self.goal_point]).flatten()



        return self._observation

    def step(self, action):
        if (self._isDiscrete):
            if self.isAbsolute_control:
                raise Exception('Error. Discrete mode is not used in Absolute Control.')

            # dv = DELTA_V  # velocity per physics step.
            # # Add noise to action
            # dv += self.np_random.normal(0.0, scale=NOISE_STD)
            # dx = [-dv, dv, 0, 0, 0, 0][action]
            # dy = [0, 0, -dv, dv, 0, 0][action]
            # dz = [0, 0, 0, 0, -dv, dv][action]
            #
            # finger_angle = 0.0  # Close the gripper
            # real_action = [dx, dy, dz, 0, finger_angle]


            dv = 0.005
            dx = [0, -dv, dv, 0, 0, 0, 0][action]
            dy = [0, 0, 0, -dv, dv, 0, 0][action]

            f = [1]
            orn = [0.708, -0.019, 0.037, 0.705]
            realAction = np.concatenate(([dx, dy, -0.002], orn, f))
        else:

            dv = 1 if self.isAbsolute_control else 0.005

            dx = action[0] * dv
            dy = action[1] * dv
            dz = action[2] * dv


            # Compute EndEffector Oritation: Oriented fixed direction
            orn = [0.713586688041687, -0.023336926475167274, 0.037783149629831314, 0.6991579532623291]
            f = [0.3]

            realAction = np.concatenate(([dx, dy, dz],  orn, f))
        return self.step2(realAction)

    def step2(self, action):
        if self.isAbsolute_control:
            self.kinova.ApplyAction_abs(action)
        else:
            print('action step: ', action)
            self.kinova.ApplyAction(action)

        for i in range(self._actionRepeat):
            p.stepSimulation()
            if self._termination():
                break
            self._envStepCounter += 1
        if self._renders:
            time.sleep(self._timeStep)
        self._observation = self._get_obs()


        done = self._termination()
        npaction = np.array(
            [action[3]])  # only penalize rotation until learning works well [action[0],action[1],action[3]])

        end_pos = self._observation[0:3]
        pos_delta = end_pos - self.goal_point

        reward = self.reward(pos_delta, npaction)

        return np.array(self._observation), reward, done, {}

    def render(self, mode="rgb_array"):
        if mode != "rgb_array":
            return np.array([])

        base_pos, orn = self._p.getBasePositionAndOrientation(self.kinova.kinovaUid)
        view_matrix = self._p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=base_pos,
            distance=self._cam_dist,
            yaw=self._cam_yaw,
            pitch=self._cam_pitch,
            roll=0,
            upAxisIndex=2)
        proj_matrix = self._p.computeProjectionMatrixFOV(
            fov=60, aspect=float(RENDER_WIDTH) / RENDER_HEIGHT,
            nearVal=0.1, farVal=100.0)
        (_, _, px, _, _) = self._p.getCameraImage(
            width=RENDER_WIDTH, height=RENDER_HEIGHT, viewMatrix=view_matrix,
            projectionMatrix=proj_matrix, renderer=self._p.ER_BULLET_HARDWARE_OPENGL)
        # renderer=self._p.ER_TINY_RENDERER)

        rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (RENDER_HEIGHT, RENDER_WIDTH, 4))

        rgb_array = rgb_array[:, :, :3]
        return rgb_array

    def _termination(self):

        return False

    def reward(self, pos_delta, action):
        reward_dist = -np.square(pos_delta).sum()
        reward_ctrl = -np.linalg.norm(action) * 10
        dist = -pos_delta
        return reward_dist+reward_ctrl


class Reach(GymWrapper):
    environment_name = 'Reach'
    entry_point = "otter.gym.bullet.reach:GymReach"
    max_episode_steps = 50
    reward_threshold = -3.75

    def __init__(self, **kwargs):
        config = {
            'image': kwargs.pop('image', False),
            'is_image_state': kwargs.pop('is_image_state', False),
            'sliding_window': kwargs.pop('sliding_window', 0),
            'random_target': kwargs.pop('random_target', False), # if the goal point is random
            'random_start': kwargs.pop('random_start', False),   # if the Jaco's position is random at the start
            'default_goal': kwargs.pop('default_goal', [0.5, 0, 0.5]),
            'image_dim': kwargs.pop('image_dim', 128),
            'urdfRoot': kwargs.pop('urdfRoot', pybullet_data.getDataPath()),
            '_timeStep':kwargs.pop('actionRepeat', 0.01),
            'actionRepeat': kwargs.pop('actionRepeat', 10),
            'isEnableSelfCollision': kwargs.pop('isEnableSelfCollision', True),
            '_render': kwargs.pop('_render', False),
            'isDiscrete': kwargs.pop('isDiscrete', False),
            'maxSteps': kwargs.pop('maxSteps', 1000),
            'hard_reset': kwargs.pop('hard_reset', False),
            'isAbsolute_control':kwargs.pop('isAbsolute_control', False),
        }
        super(Reach, self).__init__(config)

    def torque_matrix(self):
        return 2 * np.eye(self.get_action_dim())

    def make_summary(self, observations, name):
        if self.image:
            pass
            # observations = T.reshape(observations, [-1] + self.image_size())
            # T.core.summary.image(name, observations)

    def is_image(self):
        return self.image

    def image_size(self):
        if self.image:
            return [self.image_dim, self.image_dim, 3]
        return None

    def cost_fn(self, s, a):
        return np.linalg.norm(s[:,-3:], axis=-1) + np.sum(np.square(a), axis=-1)
