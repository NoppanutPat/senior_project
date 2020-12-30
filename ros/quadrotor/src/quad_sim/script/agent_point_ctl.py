#!/usr/bin/env python3

import rospy
import gym
import numpy as np

from stable_baselines3 import TD3
from stable_baselines3.td3.policies import MlpPolicy
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise

import sys
sys.path.append("/home/pat/drone/senior_project/ros/quadrotor/src/quad_sim/environment")

from quad_point_ctl import QuadEnv_point

from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.evaluation import evaluate_policy

is_load = False
weight_path = ""

if len(sys.argv) == 3:
    if sys.argv[1] == 'load':
        is_load = True
        weight_path = "/home/pat/drone/senior_project/ros/quadrotor/src/quad_sim/weight/QuadEnv-V0-episode"+sys.argv[2]+".zip"
        print("Load : "+ weight_path)
    else:
        print("Uncomplete command")
        exit()
elif len(sys.argv) == 1:
    pass
else:
    print("Uncomplete command")
    exit()

env = QuadEnv_point()

n_actions = env.action_space.shape[-1]

action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

model = TD3(MlpPolicy, env, action_noise=action_noise, verbose=1, tensorboard_log="/home/pat/drone/senior_project/ros/quadrotor/src/quad_sim/log/tensorboard",learning_rate=0.005)

if is_load:

    model = model.load(weight_path)

    model.set_env(env)


for i in range(1,1000):

    model.learn(total_timesteps=10000, log_interval=100,tb_log_name=("TD3_episode"+str(i)), reset_num_timesteps=True)

    model.save("/home/pat/drone/senior_project/ros/quadrotor/src/quad_sim/weight/QuadEnv-V0-episode"+str(i))
