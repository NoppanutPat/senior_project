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

from stable_baselines3.common.evaluation import evaluate_policy


env = QuadEnv_point()
n_actions = env.action_space.shape[-1]
action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

model = TD3(MlpPolicy, env, action_noise=action_noise, verbose=1,learning_rate=0.01)

model = model.load("/home/pat/drone/senior_project/ros/quadrotor/src/quad_sim/weight/QuadEnv-V0-episode374.zip")
# mean_reward, std_reward = evaluate_policy(model, model.get_env(), n_eval_episodes=10)

# print("mean reward : ",mean_reward)
# print("std reward : ",std_reward)

obs = env.reset()

while True:        

    action, _states = model.predict(obs, deterministic=True)
    obs, reward, done, info = env.step(action)

    if done:
      obs = env.reset()