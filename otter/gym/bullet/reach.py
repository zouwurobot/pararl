import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
#print ("current_dir=" + currentdir)
os.sys.path.insert(0,currentdir)

import abc
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
# from deepx import *
# import cv2

largeValObservation = 100

RENDER_HEIGHT = 480
RENDER_WIDTH = 480

N_DISCRETE_ACTIONS = 6 # depends on tasks
DELTA_V = 0.01
NOISE_STD_DISCRETE = 0.002  # Add noise to actions, so the env is not fully deterministic  0.01

DELTA_V_CONTINUOUS = 0.005  # velocity per physics step (for continuous relative control).
NOISE_STD_CONTINUOUS = 0.0003 # noise standard deviation (for continuous relative control)
NOISE_STD_ABSOLUTE_ACTION = 0.0005 # noise standard deviation(for absolute control )


dv = DELTA_V  # velocity per physics step.


# the first camera paremeters
CAMERA_TARGET_POS = (0.0, -0.55, 0.35)
CAM_DIS = 1.3
CAM_YAW  = 180
CAM_PITCH  = -40
CAM_ROLL = 0

# the second camera paremeters, only if multi-view is True
CAMERA_TARGET_POS_2RD = (0.0, -0.55, 0.35)
CAM_DIS_2RD = 1.3
CAM_YAW_2RD  = 180
CAM_PITCH_2RD  = -40
CAM_ROLL_2RD = 0

from ..gym_wrapper import GymWrapper

__all__ = ['Reach', 'GymReach']


class GymReach(gym.Env, metaclass=abc.ABCMeta,):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 50
    }

    def __init__(self,
                 isDiscrete= False,
                 isAbsolute_control = False,
                 timeStep = 0.01,
                 urdfRoot = pybullet_data.getDataPath(),
                 actionRepeat = 10,
                 isEnableSelfCollision = True,
                 isRender = False,
                 maxSteps = 1000,
                 debug = False,
                 multi_view = False,
                 hard_reset = True,
                 isImageObservation = True,
                 random_target=False,
                 default_goal = (0,0,0),
                 random_init_arm_angle = False,
                 *args, **kwargs):
        self.__dict__.update(kwargs)
        self._isDiscrete = isDiscrete
        self._isAbsolute_control = isAbsolute_control
        self._timeStep = timeStep
        self._urdfRoot =  urdfRoot
        self._actionRepeat =  actionRepeat
        self._isEnableSelfCollision =  isEnableSelfCollision

        self._renders = isRender
        self._maxSteps = maxSteps
        self._debug = debug
        self._multi_view = multi_view

        self._isImageObservation = isImageObservation
        self._random_target = random_target
        self._default_goal = default_goal
        self._random_init_arm_angle = random_init_arm_angle


        self.terminated = 0
        self._observation = []
        self._envStepCounter = 0
        self.hard_reset = hard_reset



        self.camera_target_pos = CAMERA_TARGET_POS
        self._cam_dist = CAM_DIS
        self._cam_yaw = CAM_YAW
        self._cam_pitch = CAM_PITCH
        self._cam_roll = CAM_ROLL
        self.renderer = p.ER_BULLET_HARDWARE_OPENGL  #p.ER_TINY_RENDERER  #


        GUI_FLAG = False
        self._p = p
        if self._renders:
            if not GUI_FLAG:
                cid = p.connect(p.SHARED_MEMORY)
                if (cid < 0):
                    cid = p.connect(p.GUI)
                else:
                    p.connect(p.DIRECT)
                p.resetDebugVisualizerCamera(1.5, 180, -41, [0.32, 0.1, -0.00])
            else:
                p.connect(p.DIRECT)

            # Debug sliders for moving the camera
            self.x_slider = p.addUserDebugParameter("x_slider", -10, 10, self.camera_target_pos[0])
            self.y_slider = p.addUserDebugParameter("y_slider", -10, 10, self.camera_target_pos[1])
            self.z_slider = p.addUserDebugParameter("z_slider", -10, 10, self.camera_target_pos[2])
            self.dist_slider = p.addUserDebugParameter("cam_dist", 0, 10, self._cam_dist)
            self.yaw_slider = p.addUserDebugParameter("cam_yaw", -200, 200, self._cam_yaw)
            self.pitch_slider = p.addUserDebugParameter("cam_pitch", -180, 180, self._cam_pitch)
        else:
            p.connect(p.DIRECT)

        self._hard_reset = True
        self._robot_urdfRoot = os.path.join(currentdir, 'assets/urdf')

        # timinglog = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "kukaTimings.json")
        self._seed()
        self.reset()

        # define Action Space
        if self._isDiscrete:
            self.action_space = spaces.Discrete(N_DISCRETE_ACTIONS)
        else:
            if self._isAbsolute_control:
                # 3 position for end-effector wrt the Cartesian Space
                action_dim = 3
                self._action_bound = 1

                # TODO action need to be scaled to the limitation of EE.
            else:
                # 3 direction for the arm movement, from -1 to 1
                action_dim = 3
                self._action_bound = 1
            action_high = np.array([self._action_bound] * action_dim)
            self.action_space = spaces.Box(-action_high, action_high, dtype=np.float32)

        # define Obsevation Space
        observationDim = len(self._get_obs())
        observation_high = np.array([largeValObservation] * observationDim)
        self.observation_space = spaces.Box(-observation_high, observation_high)
        self.viewer = None

        self._hard_reset = self.hard_reset  # This assignment need to be after reset()

    def reset(self):
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
            self.view_matrix1 = p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=self.camera_target_pos,
                distance=self._cam_dist,
                yaw=self._cam_yaw,
                pitch=self._cam_pitch,
                roll=self._cam_roll,
                upAxisIndex=2)
            self.proj_matrix1 = p.computeProjectionMatrixFOV(
                fov=60, aspect=float(RENDER_WIDTH) / RENDER_HEIGHT,
                nearVal=0.1, farVal=100.0)

            if self._multi_view:
                # adding a second camera on the other side of the robot
                self.view_matrix2 = p.computeViewMatrixFromYawPitchRoll(
                    cameraTargetPosition= CAMERA_TARGET_POS_2RD,
                    distance= CAM_DIS_2RD,
                    yaw= CAM_YAW_2RD,
                    pitch= CAM_PITCH_2RD,
                    roll=CAM_ROLL_2RD,
                    upAxisIndex=2)
                self.proj_matrix2 = p.computeProjectionMatrixFOV(
                    fov=60, aspect=float(RENDER_WIDTH) / RENDER_HEIGHT,
                    nearVal=0.1, farVal=100.0)

            self.robot_init_pos = self.get_init_joint_angle()
            self.goal_point = self.get_target_pos()

            # load the kinova
            self.kinova = kinova.Kinova(p,  robot_type='j2s7s300',
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
    def get_init_joint_angle(self):
        if self._random_init_arm_angle:
            #TODO write a random function for start positon
            #start_pos = np.random.uniform(low=-np.pi, high=np.pi, size=self.model.nq)
            raise print('random function is not completed!')
        else:

            start_pos = [-4.54, 3.438, 9.474, 0.749, 4.628, 4.472, 5.045, 1, 1, 1]
            start_pos = [-7.624, 2.814, 12.568, 0.758, -1.647, 4.492, 5.025, 1, 1, 1] # home position
            start_pos = [-7.81, 3.546, 12.883, 0.833, -2.753, 4.319, 5.917 ,1, 1, 1]  # init position

        return start_pos

    def get_target_pos(self):
        if self._random_target:
            goal = np.array([self.np_random.uniform(-0.2, 0.2), self.np_random.uniform(-0.2, 0.2)])
        else:
            goal = np.array(self._default_goal)
        return goal

    def __del__(self):
        p.disconnect()

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _get_obs(self):

        image_obs = self.render("rgb_array")

        if self._isImageObservation:
            self._observation = image_obs.flatten()
        else:
            kinovaState = self.kinova.GetObservation()
            self._observation = \
                np.concatenate([np.array(kinovaState),
                                np.array(kinovaState[:3]) - self.goal_point]).flatten()
        return self._observation

    def step(self, action):

        finger_angle = [0.0]  # Close the gripper
        orn = [0.708, -0.019, 0.037, 0.705]

        if (self._isDiscrete):
            if self._isAbsolute_control:
                raise Exception('Error. Discrete mode is not used in Absolute Control.')
            dv = DELTA_V  # velocity per physics step.
            # Add noise to action
            dv += self.np_random.normal(0.0, scale=NOISE_STD_DISCRETE)
            dx = [-dv, dv, 0, 0, 0, 0][action]
            dy = [0, 0, -dv, dv, 0, 0][action]
            dz = [0, 0, 0, 0, -dv, dv][action]

            realAction = np.concatenate(([dx, dy, dz], orn, finger_angle))
        else:
            if self._isAbsolute_control:
                action += self.np_random.normal(0.0, scale=NOISE_STD_ABSOLUTE_ACTION)
                realAction = np.concatenate((action, orn, finger_angle))
            else:
                dv = DELTA_V_CONTINUOUS
                # Add noise to action
                dv += self.np_random.normal(0.0, scale=NOISE_STD_CONTINUOUS)

                dx = action[0] * dv
                dy = action[1] * dv
                dz = action[2] * dv

                realAction = np.concatenate(([dx, dy, dz], orn, finger_angle))

        return self.step2(realAction)

    def step2(self, action):
        for i in range(self._actionRepeat):
            if self._isAbsolute_control:
                self.kinova.ApplyAction_EndEffectorPose(action)
            else:
                self.kinova.ApplyAction(action)

            p.stepSimulation()
            if self._termination():
                break
            self._envStepCounter += 1
        if self._renders:
            time.sleep(self._timeStep)
        self._observation = self._get_obs()


        done = self._termination()
        # npaction = np.array(
        #     [action[3]])  # only penalize rotation until learning works well [action[0],action[1],action[3]])

        #end_pos = self._observation[0:3]
        #pos_delta = end_pos - self.goal_point

        reward = self._reward(self._observation, action)

        return np.array(self._observation), reward, done, {}

    def render(self, mode="human",  close=False):
        if mode != "rgb_array":
            return np.array([])


        if self._debug:
            self._cam_dist = p.readUserDebugParameter(self.dist_slider)
            self._cam_yaw = p.readUserDebugParameter(self.yaw_slider)
            self._cam_pitch = p.readUserDebugParameter(self.pitch_slider)
            x = p.readUserDebugParameter(self.x_slider)
            y = p.readUserDebugParameter(self.y_slider)
            z = p.readUserDebugParameter(self.z_slider)
            camera_target_pos = (x, y, z)
            #self._cam_roll = p.readUserDebugParameter(self.roll_slider)

            # TODO: recompute view_matrix and proj_matrix only in debug mode
            self.view_matrix1 = p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=camera_target_pos,
                distance=self._cam_dist,
                yaw=self._cam_yaw,
                pitch=self._cam_pitch,
                roll=self._cam_roll,
                upAxisIndex=2)
            self.proj_matrix1 = p.computeProjectionMatrixFOV(
                fov=60, aspect=float(RENDER_WIDTH) / RENDER_HEIGHT,
                nearVal=0.1, farVal=100.0)

        (_, _, px1, _, _) = p.getCameraImage(width=RENDER_WIDTH, height=RENDER_HEIGHT, viewMatrix=self.view_matrix1,
                                             projectionMatrix=self.proj_matrix1, renderer=self.renderer)
        rgb_array1 = np.array(px1, dtype=np.uint8)

        if self._multi_view:
            # adding a second camera on the other side of the robot
            (_, _, px2, _, _) = p.getCameraImage(width=RENDER_WIDTH, height=RENDER_HEIGHT, viewMatrix=self.view_matrix2,
                                                    projectionMatrix=self.proj_matrix2, renderer=self.renderer)
            rgb_array2 = np.array(px2)
            rgb_array_res = np.concatenate((rgb_array1[:, :, :3], rgb_array2[:, :, :3]), axis=2)
        else:
            rgb_array_res = rgb_array1[:, :, :3]


        return rgb_array_res

    def _termination(self):
        if self.terminated or self._envStepCounter > self._maxSteps:
            self._observation = self._get_obs()
            return True
        return False

    def _reward(self, obs, action):
        reward_dist = -np.square(obs).sum()
        reward_ctrl = -np.linalg.norm(action) * 10
        dist = -obs
        return 0


class KinovaReacher(GymReach):
    def __init__(self, **kwargs):

        GymReach.__init__(self,  **kwargs)

    def _reward(self, obs, action):

        # if contact_with_table or self.n_contacts >= N_CONTACTS_BEFORE_TERMINATION \
        #         or self.n_steps_outside >= N_STEPS_OUTSIDE_SAFETY_SPHERE:
        #     self.terminated = True
        return 1.111

class Reach(GymWrapper):
    environment_name = 'Reach'
    entry_point = "otter.gym.bullet.reach:KinovaReacher"
    max_episode_steps = 1000
    reward_threshold = -3.75

    def __init__(self, **kwargs):
        config = {
            'isDiscrete': kwargs.pop('isDiscrete', False),
            'isAbsolute_control':kwargs.pop('isAbsolute_control', False),
            'timeStep': kwargs.pop('timeStep', 0.01),
            'actionRepeat': kwargs.pop('actionRepeat', 10),
            'isEnableSelfCollision': kwargs.pop('isEnableSelfCollision', True),
            'urdfRoot': kwargs.pop('urdfRoot', pybullet_data.getDataPath()),
            'isRender': kwargs.pop('isRender', True),
            'maxSteps': kwargs.pop('maxSteps', 1000),
            'debug': kwargs.pop('debug', True),
            'multi_view': kwargs.pop('multi_view', False),
            'hard_reset': kwargs.pop('hard_reset', False),

            'isImageObservation': kwargs.pop('isImageObservation', False),
            'random_target': kwargs.pop('random_target', False),
            'random_init_arm_angle': kwargs.pop('random_init_arm_angle', False),
            'default_goal': kwargs.pop('default_goal', [0.5, 0, 0.5]),

            'image': kwargs.pop('image', True),
            'sliding_window': kwargs.pop('sliding_window', 0),
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

# class Reach(GymWrapper):
#     environment_name = 'Reach'
#     entry_point = "otter.gym.bullet.reach:GymReach"
#     max_episode_steps = 50
#     reward_threshold = -3.75
#
#     def __init__(self, **kwargs):
#         config = {
#             'isDiscrete': kwargs.pop('isDiscrete', False),
#             'isAbsolute_control':kwargs.pop('isAbsolute_control', False),
#             'timeStep': kwargs.pop('timeStep', 0.01),
#             'actionRepeat': kwargs.pop('actionRepeat', 10),
#             'isEnableSelfCollision': kwargs.pop('isEnableSelfCollision', True),
#             'urdfRoot': kwargs.pop('urdfRoot', pybullet_data.getDataPath()),
#             'isRender': kwargs.pop('isRender', True),
#             'maxSteps': kwargs.pop('maxSteps', 1000),
#             'debug': kwargs.pop('debug', True),
#             'multi_view': kwargs.pop('multi_view', False),
#             'hard_reset': kwargs.pop('hard_reset', False),
#
#             'isImageObservation': kwargs.pop('isImageObservation', False),
#             'random_target': kwargs.pop('random_target', False),
#             'random_init_arm_angle': kwargs.pop('random_init_arm_angle', False),
#             'default_goal': kwargs.pop('default_goal', [0.5, 0, 0.5]),
#
#             'image': kwargs.pop('image', True),
#             'sliding_window': kwargs.pop('sliding_window', 0),
#         }
#         super(Reach, self).__init__(config)
#
#     def torque_matrix(self):
#         return 2 * np.eye(self.get_action_dim())
#
#     def make_summary(self, observations, name):
#         if self.image:
#             pass
#             # observations = T.reshape(observations, [-1] + self.image_size())
#             # T.core.summary.image(name, observations)
#
#     def is_image(self):
#         return self.image
#
#     def image_size(self):
#         if self.image:
#             return [self.image_dim, self.image_dim, 3]
#         return None
#
#     def cost_fn(self, s, a):
#         return np.linalg.norm(s[:,-3:], axis=-1) + np.sum(np.square(a), axis=-1)


