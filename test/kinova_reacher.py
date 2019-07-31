from collections import OrderedDict
import numpy as np
from gym.spaces import Box, Dict


from otter.gym.bullet.reach import GymReach
from otter.core.multitask_env import MultitaskEnv


class SawyerReachXYZEnv(GymReach, MultitaskEnv):

    def __init__(
            self,
            reward_type='hand_distance',
            norm_order=1,
            indicator_threshold=0.06,

            fix_goal=False,
            fixed_goal=(0.15, 0.6, 0.3),
            hide_goal_markers=False,

            **kwargs
    ):
        MultitaskEnv.__init__(self)
        GymReach.__init__(self, **kwargs)

        self.reward_type = reward_type
        self.norm_order = norm_order
        self.indicator_threshold = indicator_threshold

        self.fix_goal = fix_goal
        self.fixed_goal = np.array(fixed_goal)
        self._state_goal = None
        self.hide_goal_markers = hide_goal_markers
        self.action_space = Box(np.array([-1, -1, -1]), np.array([1, 1, 1]), dtype=np.float32)
        # self.hand_space = Box(self.hand_low, self.hand_high, dtype=np.float32)
        # self.observation_space = Dict([
        #     ('observation', self.hand_space),
        #     ('desired_goal', self.hand_space),
        #     ('achieved_goal', self.hand_space),
        #     ('state_observation', self.hand_space),
        #     ('state_desired_goal', self.hand_space),
        #     ('state_achieved_goal', self.hand_space),
        #     ('proprio_observation', self.hand_space),
        #     ('proprio_desired_goal', self.hand_space),
        #     ('proprio_achieved_goal', self.hand_space),
        # ])
        self.reset()

    # def step(self, action):
    #     return super().step(action)

    def step(self, action):
        self.set_xyz_action(action)
        # keep gripper closed
        self.do_simulation(np.array([1]))
        # The marker seems to get reset every time you do a simulation
        self._set_goal_marker(self._state_goal)
        ob = self._get_obs()
        reward = self.compute_reward(action, ob)
        info = self._get_info()
        done = False
        return ob, reward, done, info


    def _get_obs(self):
        flat_obs = self.get_endeff_pos()
        return dict(
            observation=flat_obs,
            desired_goal=self._state_goal,
            achieved_goal=flat_obs,
            state_observation=flat_obs,
            state_desired_goal=self._state_goal,
            state_achieved_goal=flat_obs,
            proprio_observation=flat_obs,
            proprio_desired_goal=self._state_goal,
            proprio_achieved_goal=flat_obs,
        )
    def compute_rewards(self, actions, obs):
            # achieved_goals = obs['state_achieved_goal']
            # desired_goals = obs['state_desired_goal']
            # hand_pos = achieved_goals
            # goals = desired_goals
            # hand_diff = hand_pos - goals
            # if self.reward_type == 'hand_distance':
            #     r = -np.linalg.norm(hand_diff, ord=self.norm_order, axis=1)
            # elif self.reward_type == 'vectorized_hand_distance':
            #     r = -np.abs(hand_diff)
            # elif self.reward_type == 'hand_success':
            #     r = -(np.linalg.norm(hand_diff, ord=self.norm_order, axis=1)
            #           > self.indicator_threshold).astype(float)
            # else:
            #     raise NotImplementedError("Invalid/no reward type.")

        print('reward is new !')
        r = 1
        return r
