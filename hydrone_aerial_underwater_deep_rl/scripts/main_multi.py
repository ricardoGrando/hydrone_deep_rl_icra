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

if __name__=="__main__":

    rospy.init_node("multi_agents_rl")

    args = build_parse()

    if args.Algorithm in ["COMA","QMIX","VDN"]:
        args.is_continous = False
    else:
        args.is_continous = True

    env = gym.make('hydrone_Circuit_Simple-v0', env_stage=1, observation_mode=0, continuous=True, goal_list=None, agent_number=0, model_path='/home/ricardo/hydrone_ws/src/hydrone_deep_rl_icra/hydrone_aerial_underwater_deep_rl/models/goal_box/model.sdf')
    env.reset()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        env.step(np.array([0.1, 0.0, 0.0]))
        rate.sleep()
