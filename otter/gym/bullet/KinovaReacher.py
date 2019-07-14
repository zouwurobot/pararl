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


from .base import  KinovaXYZ
from ..gym_wrapper import GymWrapper

__all__ = [ 'ImageKinovaReacherXYZEnv']

class ImageKinovaReacherXYZ(KinovaXYZ):
    def __init__(self, **kwargs):
        KinovaXYZ.__init__(self,  **kwargs)

    def reset(self):
        self.goal_point = self.get_target_pos()
        print('reset goal point:', self.goal_point)
        return  super().reset()


    def get_target_pos(self):
        if self._random_target:
            goal_point = np.array([self.np_random.uniform(-0.2, 0.2), self.np_random.uniform(-0.2, 0.2)])
        else:
            goal_point = np.array(self._default_goal)
        return  goal_point

    def _reward(self, obs, action):

        # if contact_with_table or self.n_contacts >= N_CONTACTS_BEFORE_TERMINATION \
        #         or self.n_steps_outside >= N_STEPS_OUTSIDE_SAFETY_SPHERE:
        #     self.terminated = True
        kinovaEEPos, _ = self.kinova.GetEndEffectorObersavations()
        dist = np.array(kinovaEEPos) - self.goal_point
        reward_dist = -np.square(dist).sum()
        #reward_ctrl = -np.linalg.norm(action) * 10
        #dist = -obs
        return reward_dist

class ImageKinovaReacherXYZEnv(GymWrapper):
    environment_name = 'ImageKinovaReacherXYZEnv-v0'
    entry_point = "otter.gym.bullet.KinovaReacher:ImageKinovaReacherXYZ"
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

            'isImageObservation': kwargs.pop('isImageObservation', True),
            'random_target': kwargs.pop('random_target', False),
            'random_init_arm_angle': kwargs.pop('random_init_arm_angle', False),
            'default_goal': kwargs.pop('default_goal', [0.5, 0, 0.5]),

            # new parameters
            'image': kwargs.pop('image', True),
            'image_dim': kwargs.pop('image_dim', 480),
            'sliding_window': kwargs.pop('sliding_window', 0),

        }
        if config['image'] is True:
            config['isImageObservation'] = True
            config['image_height'] = config['image_width'] = config['image_dim']
        super(ImageKinovaReacherXYZEnv, self).__init__(config)

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