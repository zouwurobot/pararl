"""
Use this script to control the env with your keyboard.
For this script to work, you need to have the PyGame window in focus.

See/modify `char_to_action` to set the key-to-action mapping.
"""
import sys
import gym

import numpy as np

import pygame
from pygame.locals import QUIT, KEYDOWN


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



import otter.gym as gym
import numpy as np

env_params = {
    "environment_name": "ImageKinovaCupPusherEnv-v0",  #ImageKinovaCupPusherEnv-v0   ImageKinovaReacherXYZEnv-v0
    "random_start": False,
    "random_target": False,

    "image": True,
    "image_dim": 128,
    "goal_point": [0.5, 0, 0.5],
    'isAbsolute_control': False,
    "reward_test":1.66
}

env = gym.from_config(env_params)

NDIM = env.gym_env.action_space.low.size
lock_action = False
t1 = time.time()
obs = env.reset()

while True:
    action = env.gym_env.action_space.sample()
    obs, reward, done, _ = env.step(action)

    print("state info :", obs.shape)
    print("action ", action)

    if done:
        t2 = time.time()
        print('total time :', t2 - t1)
        obs = env.reset()

    env.render()
    print( 'reward :', reward)
    print("-------------------------")


