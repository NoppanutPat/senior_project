#!/usr/bin/env python3

import rospy
import gym
import numpy as np

from stable_baselines3 import TD3
from stable_baselines3.td3.policies import MlpPolicy
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise

import sys
sys.path.append("/home/pat/drone/senior_project/ros/quadrotor/src/quad_sim/environment")

from quadrotor_env import QuadEnv

# from stable_baselines3.common.env_checker import check_env

env = QuadEnv()

# check_env(env)

n_actions = env.action_space.shape[-1]

action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

model = TD3(MlpPolicy, env, action_noise=action_noise, verbose=1, tensorboard_log="/home/pat/drone/senior_project/ros/quadrotor/src/quad_sim/log/tensorboard",learning_rate=0.01)

for i in range(1,100):

    model.learn(total_timesteps=10000, log_interval=50,tb_log_name=("TD3_episode"+str(i)))

    model.save("/home/pat/drone/senior_project/ros/quadrotor/src/quad_sim/weight/QuadEnv-V0-episode"+str(i))
