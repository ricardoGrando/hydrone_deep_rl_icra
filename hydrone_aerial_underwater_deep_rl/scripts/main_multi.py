#!/usr/bin/env python3

import rospy
import os
import pickle
import argparse
import sys
import numpy as np
import copy
import gym
import gym_hydrone

from multi.common.arguments import *
from multi.agent import MADDPG, COMA, MAAC#, QMIX, MASoftQ, VDN, MATD3, FACMAC, FOP
from multi.network.base_net import DiscreteMLP, DiscreteGaussianNet, ContinuousMLP, RNN, ContGaussianNet

if __name__=="__main__":

    rospy.init_node("multi_agents_rl")

    args = build_parse()

    if args.Algorithm in ["COMA","QMIX","VDN"]:
        args.is_continous = False
    else:
        args.is_continous = True

    # must be equal for all agents, 20 lidar, distance goal, angle x-y, angle z-distance,
    lidar_samples = 20
    envs = []
    envs.append(gym.make('hydrone_Circuit_Simple-v0', env_stage=1, observation_mode=0, continuous=True, lidar_samples=lidar_samples, agent_number=0, goal_list=None, model_path='/home/ricardo/hydrone_ws/src/hydrone_deep_rl_icra/hydrone_aerial_underwater_deep_rl/models/goal_box/model0.sdf'))
    envs.append(gym.make('hydrone_Circuit_Simple-v0', env_stage=1, observation_mode=0, continuous=True, lidar_samples=lidar_samples, agent_number=1, goal_list=None, model_path='/home/ricardo/hydrone_ws/src/hydrone_deep_rl_icra/hydrone_aerial_underwater_deep_rl/models/goal_box/model1.sdf'))
    envs.append(gym.make('hydrone_Circuit_Simple-v0', env_stage=1, observation_mode=0, continuous=True, lidar_samples=lidar_samples, agent_number=2, goal_list=None, model_path='/home/ricardo/hydrone_ws/src/hydrone_deep_rl_icra/hydrone_aerial_underwater_deep_rl/models/goal_box/model2.sdf'))
    envs.append(gym.make('hydrone_Circuit_Simple-v0', env_stage=1, observation_mode=0, continuous=True, lidar_samples=lidar_samples, agent_number=3, goal_list=None, model_path='/home/ricardo/hydrone_ws/src/hydrone_deep_rl_icra/hydrone_aerial_underwater_deep_rl/models/goal_box/model3.sdf'))
    envs.append(gym.make('hydrone_Circuit_Simple-v0', env_stage=1, observation_mode=0, continuous=True, lidar_samples=lidar_samples, agent_number=4, goal_list=None, model_path='/home/ricardo/hydrone_ws/src/hydrone_deep_rl_icra/hydrone_aerial_underwater_deep_rl/models/goal_box/model4.sdf'))

    for i in range (0, len(envs)):
        envs[i].reset()

    args = get_env_parameters(args,envs)

    if args.Algorithm == "MADDPG":
        args = get_maddpg_args(args)
        trainer = MADDPG.MADDPG(args = args,policy = ContinuousMLP)

    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     env.step(np.array([0.1, 0.0, 0.0]))
    #     rate.sleep()
