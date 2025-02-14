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
import matplotlib.pyplot as plt
from multi.common.arguments import *
from multi.agent import MADDPG, COMA, MAAC#, QMIX, MASoftQ, VDN, MATD3, FACMAC, FOP
from multi.network.base_net import DiscreteMLP, DiscreteGaussianNet, ContinuousMLP, RNN, ContGaussianNet

def train(args,envs,trainer):

    best_reward = -np.inf
    total_reward = []
    avg_reward_list = []
    os.makedirs("/home/ricardo/config/saves/rl_rewards/" +args.Environment, exist_ok=True)
    os.makedirs("/home/ricardo/config/saves/images/" +args.Environment, exist_ok=True)
    sample_added = 0
    for i in range(args.n_episodes):

        for agent in range (0, args.n_agents):
            observation = {envs[agent].reset() for agent in range (0, args.n_agents)}

        print(envs[0].initGoal)
        print(observation)
        break

    #     global_reward = 0
    #
    #     while True:
    #
    #         state = env.state()
    #         action = trainer.choose_action(observation)
    #
    #         next_observation,rwd,termination,truncation,info = env.step(action)
    #         next_state = env.state()
    #         global_reward += sum(list(rwd.values()))
    #
    #         if args.Algorithm in ["VDN","QMIX","FACMAC","FOP"]:
    #             reward = global_reward
    #         else:
    #             reward = rwd
    #
    #         sample_added += 1
    #         if all(list(termination.values())) or all(list(truncation.values())):
    #
    #             done = {}
    #             for key in termination.keys():
    #                 done[key] = True
    #
    #             trainer.add(state,observation,action,reward,next_state,next_observation,done)
    #
    #             if args.Algorithm in ["COMA"]:
    #                 if i%args.train_network == 0:
    #                     trainer.learn()
    #
    #             break
    #
    #         else:
    #             trainer.add(state,observation,action,reward,next_state,next_observation,termination)
    #
    #         if args.Algorithm in ["MADDPG","MASoftQ","VDN","MATD3","QMIX","FACMAC"]:
    #             trainer.learn()
    #
    #         observation = next_observation
    #
    #     total_reward.append(global_reward)
    #     avg_reward = np.mean(total_reward[-40:])
    #
    #     if avg_reward>best_reward and i > 10:
    #         best_reward=avg_reward
    #         if args.save_rl_weights:
    #             print("Weights Saved !!!")
    #             trainer.save(args.Environment)
    #
    #     print("Episode * {} * Avg Reward is ==> {}".format(i, avg_reward))
    #     avg_reward_list.append(avg_reward)
    #
    # if args.save_results:
    #     f = open("/home/ricardo/config/saves/rl_rewards/" +args.Environment + "/" + args.Algorithm + ".pkl","wb")
    #     pickle.dump(avg_reward_list,f)
    #     f.close()
    #
    # plt.figure()
    # plt.title(f"Reward values - {args.Algorithm}")
    # plt.xlabel("Iterations")
    # plt.ylabel("Reward")
    # plt.plot(avg_reward_list)
    # plt.show()

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

    train(args,envs,trainer)

    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     env.step(np.array([0.1, 0.0, 0.0]))
    #     rate.sleep()
