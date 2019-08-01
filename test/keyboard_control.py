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

pygame.init()
screen = pygame.display.set_mode((400, 300))


char_to_action = {
    'w': np.array([0, -1, 0, 0]),
    'a': np.array([1, 0, 0, 0]),
    's': np.array([0, 1, 0, 0]),
    'd': np.array([-1, 0, 0, 0]),

    'q': np.array([1, -1, 0, 0]),
    'e': np.array([-1, -1, 0, 0]),
    'z': np.array([1, 1, 0, 0]),
    'c': np.array([-1, 1, 0, 0]),
    'k': np.array([0, 0, 1, 0]),
    'j': np.array([0, 0, -1, 0]),


    'h': 'close',
    'l': 'open',
    'x': 'toggle',
    'r': 'reset',
    'p': 'put obj in hand',
}
import pygame

import otter.gym as gym
import numpy as np

env_params = {
    "environment_name": "ImageKinovaCupPusherEnv-v0",

    "random_init_cup_position": True,
    "random_target_position": True,
    "debug": True,
    "image": True,
    "image_dim": 64,

    'isRender': True,

    'debug': True
}

data_params=dict(
        num_rollouts=10,
        init_std=0.5,
        smooth_noise=False,
    )
horizon = 50

env = gym.from_config(env_params)

NDIM = env.gym_env.action_space.low.size
lock_action = False
t1 = time.time()
obs = env.reset()

action = np.zeros(10)
while True:
    done = False
    if not lock_action:
        action[:3] = 0
    for event in pygame.event.get():
        event_happened = True
        if event.type == QUIT:
            sys.exit()
        if event.type == KEYDOWN:
            char = event.dict['key']
            new_action = char_to_action.get(chr(char), None)
            if new_action == 'toggle':
                lock_action = not lock_action
            elif new_action == 'reset':
                done = True
            elif new_action == 'close':
                action[3] = 1
            elif new_action == 'open':
                action[3] = -1
            elif new_action == 'put obj in hand':
                print("putting obj in hand")
                env.put_obj_in_hand()
                action[3] = 1
            elif new_action is not None:
                action[:3] = new_action[:3]
            else:
                action = np.zeros(3)
    #print('______________________')
    #print('actions: ', action)
    obs, reward, done, _ = env.step(action[:3])

    #print("state info :", obs.shape)

    if done:
        t2 = time.time()
        print('total time :', t2 - t1)
        obs = env.reset()

    env.render()
    #print( 'reward :', reward)


