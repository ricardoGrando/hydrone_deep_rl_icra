import gym
import gym_hydrone
import rospy
import os

env = gym.make('hydrone_Circuit_Simple-v0', env_stage=1, observation_mode=0, continuous=True, goal_list=None, agent_number=0, model_path='/home/ricardo/hydrone_ws/src/hydrone_deep_rl_icra/hydrone_aerial_underwater_deep_rl/models/goal_box/model.sdf')
