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
    '0': 0,
    '1': 1,
    '2': 2,
    '3': 3,

    '4': 4,
    '5': 5,
    '6': 6,
    '7': 7,

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
    "environment_name": "ImageKinovaReacherXYZEnv-v0",
    "isDiscrete":True,
    "isAbsolute_control":False,
    "random_start": False,
    "random_target": False,

    "image": True,
    "image_dim": 128,
    "goal_point": [0.5, 0, 0.5],
    'isAbsolute_control': False,

}


env = gym.from_config(env_params)

#NDIM = env.gym_env.action_space.low.size
lock_action = False
obs = env.reset()
action = 0
while True:
    done = False
    if not lock_action:
        action  = 0
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


            elif new_action is not None:
                action  = new_action
            else:
                action = 0
    print('______________________')
    print('actions: ', action)
    obs, reward, done, _ = env.step(action)
    if done:
        obs = env.reset()
    env.render()
    print( 'reward :', reward)