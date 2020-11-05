#!/usr/bin/env python

import rospy
import os
import json
import numpy as np
import random
import time
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from collections import deque
from std_msgs.msg import *
from environment_3D import Env
import torch
import torch.nn.functional as F
import gc
import torch.nn as nn
import math
from collections import deque
import copy
import math

#---Directory Path---#
dirPath = os.path.dirname(os.path.realpath(__file__))
#---Functions to make network updates---#
 
def soft_update(target, source, tau):
    for target_param, param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(target_param.data*(1.0 - tau)+ param.data*tau)

def hard_update(target,source):
    for target_param, param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(param.data)

#---Ornstein-Uhlenbeck Noise for action---#

class OUNoise(object):
    def __init__(self, action_space, mu=0.0, theta=0.15, max_sigma=0.99, min_sigma=0.2, decay_period=1000000):
        self.mu           = mu
        self.theta        = theta
        self.sigma        = max_sigma
        self.max_sigma    = max_sigma
        self.min_sigma    = min_sigma
        self.decay_period = decay_period
        self.action_dim   = action_space
        self.reset()
        
    def reset(self):
        self.state = np.ones(self.action_dim) * self.mu
        
    def evolve_state(self):
        x  = self.state
        dx = self.theta * (self.mu - x) + self.sigma * np.random.randn(self.action_dim)
        self.state = x + dx
        return self.state
    
    def get_noise(self, t=0): 
        ou_state = self.evolve_state()
        decaying = float(float(t)/ self.decay_period)
        self.sigma = max(self.sigma - (self.max_sigma - self.min_sigma) * min(1.0, decaying), self.min_sigma)
        return ou_state

#---Critic--#

class Critic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Critic, self).__init__()
        
        self.state_dim = state_dim
        self.action_dim = action_dim
        
        # self.fc1 = nn.Linear(state_dim, 256)
        # nn.init.xavier_uniform_(self.fc1.weight)
        # self.fc1.bias.data.fill_(0.01)
        
        # self.fa1 = nn.Linear(action_dim, 256)
        # nn.init.xavier_uniform_(self.fa1.weight)
        # self.fa1.bias.data.fill_(0.01)
        # # self.fa1.weight.data.uniform_(-EPS, EPS)
        # # self.fa1.bias.data.uniform_(-EPS, EPS)
        
        self.fca1 = nn.Linear(self.state_dim + self.action_dim, 512)
        nn.init.xavier_uniform_(self.fca1.weight)
        self.fca1.bias.data.fill_(0.01)
        # self.fca1.weight.data.uniform_(-EPS, EPS)
        # self.fca1.bias.data.uniform_(-EPS, EPS)
        
        self.fca2 = nn.Linear(512, 512)
        nn.init.xavier_uniform_(self.fca2.weight)
        self.fca2.bias.data.fill_(0.01)
        # self.fca2.weight.data.uniform_(-EPS, EPS)
        # self.fca2.bias.data.uniform_(-EPS, EPS)

        self.fca3 = nn.Linear(512, 1)
        nn.init.xavier_uniform_(self.fca3.weight)
        self.fca3.bias.data.fill_(0.01)
        # self.fca2.weight.data.uniform_(-EPS, EPS)
        # self.fca2.bias.data.uniform_(-EPS, EPS)

        # self.fca4 = nn.Linear(512, 1)
        # nn.init.xavier_uniform_(self.fca4.weight)
        # self.fca4.bias.data.fill_(0.01)
        # # self.fca2.weight.data.uniform_(-EPS, EPS)
        # # self.fca2.bias.data.uniform_(-EPS, EPS)
        
    def forward(self, state, action):
        # xs = torch.relu(self.fc1(state))
        # xa = torch.relu(self.fa1(action))
        # x = torch.cat((xs,xa), dim=1)
        x_state_action = torch.cat([state, action], 1)
        x = torch.relu(self.fca1(x_state_action))
        x = torch.relu(self.fca2(x))
        # x = torch.relu(self.fca3(x))
        vs = self.fca3(x)
        return vs

#---Actor---#

class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, action_limit_v, action_limit_w):
        super(Actor, self).__init__()
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.action_limit_v = action_limit_v
        self.action_limit_w = action_limit_w
        
        self.fa1 = nn.Linear(state_dim, 512)
        nn.init.xavier_uniform_(self.fa1.weight)
        self.fa1.bias.data.fill_(0.01)
        # self.fa1.weight.data.uniform_(-EPS, EPS)
        # self.fa1.bias.data.uniform_(-EPS, EPS)
        
        self.fa2 = nn.Linear(512, 512)
        nn.init.xavier_uniform_(self.fa2.weight)
        self.fa2.bias.data.fill_(0.01)
        # self.fa2.weight.data.uniform_(-EPS, EPS)
        # self.fa2.bias.data.uniform_(-EPS, EPS)
        
        self.fa3 = nn.Linear(512, action_dim)
        nn.init.xavier_uniform_(self.fa3.weight)
        self.fa3.bias.data.fill_(0.01)
        # self.fa3.weight.data.uniform_(-EPS, EPS)
        # self.fa3.bias.data.uniform_(-EPS, EPS)

        # self.fa4 = nn.Linear(512, 512)
        # nn.init.xavier_uniform_(self.fa4.weight)
        # self.fa4.bias.data.fill_(0.01)
        # # self.fa3.weight.data.uniform_(-EPS, EPS)
        # # self.fa3.bias.data.uniform_(-EPS, EPS)

        # self.fa5 = nn.Linear(512, action_dim)
        # nn.init.xavier_uniform_(self.fa5.weight)
        # self.fa5.bias.data.fill_(0.01)
        # # self.fa3.weight.data.uniform_(-EPS, EPS)
        # # self.fa3.bias.data.uniform_(-EPS, EPS)
        
    def forward(self, state):
        x = torch.relu(self.fa1(state))
        x = torch.relu(self.fa2(x))
        # x = torch.relu(self.fa3(x))
        # x = torch.relu(self.fa4(x))
        action = self.fa3(x).squeeze(0)
        # rospy.loginfo(" %s ", str(action))
        if state.shape <= torch.Size([self.state_dim]):
            action[0] = ((torch.tanh(action[0]) + 1.0)/2.0)*self.action_limit_v
            action[1] = torch.tanh(action[1])*self.action_limit_w
            action[2] = torch.tanh(action[2])*self.action_limit_w
        else:
            action[:,0] = ((torch.tanh(action[:,0]) + 1.0)/2.0)*self.action_limit_v
            action[:,1] = torch.tanh(action[:,1])*self.action_limit_w
            action[:,2] = torch.tanh(action[:,2])*self.action_limit_w
        return action

#---Memory Buffer---#

class ReplayBuffer:
    def __init__(self, capacity):
        self.capacity = capacity
        self.buffer = []
        self.position = 0
    
    def push(self, state, action, reward, next_state, done):
        if len(self.buffer) < self.capacity:
            self.buffer.append(None)
        self.buffer[self.position] = (state, action, reward, next_state, done)
        self.position = (self.position + 1) % self.capacity
    
    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        state, action, reward, next_state, done = map(np.stack, zip(*batch))
        return state, action, reward, next_state, done
    
    def __len__(self):
        return len(self.buffer)

class Trainer:    
    def __init__(self, state_dim, action_dim, action_limit_v, action_limit_w, replay_buffer):
        
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.action_limit_v = action_limit_v
        self.action_limit_w = action_limit_w
        #print('w',self.action_limit_w)       

        self.replay_buffer = replay_buffer

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu") 
        
        self.actor = Actor(self.state_dim, self.action_dim, self.action_limit_v, self.action_limit_w).to(device=self.device)
        self.target_actor = Actor(self.state_dim, self.action_dim, self.action_limit_v, self.action_limit_w).to(device=self.device)
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), LEARNING_RATE)
        
        self.critic = Critic(self.state_dim, self.action_dim).to(device=self.device)
        self.target_critic = Critic(self.state_dim, self.action_dim).to(device=self.device)
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), LEARNING_RATE)
        self.pub_qvalue = rospy.Publisher('qvalue', Float32, queue_size=5)
        self.qvalue = Float32()
        
        hard_update(self.target_actor, self.actor)
        hard_update(self.target_critic, self.critic)
        
    def get_exploitation_action(self,state):
        state = torch.FloatTensor(state).to(self.device)
        action = self.actor.forward(state)
        #print('actionploi', action)
        # rospy.loginfo(" %s ", str(action))
        return action.detach().cpu().numpy()
        
    def get_exploration_action(self, state):
        state = torch.FloatTensor(state).to(self.device)
        action = self.actor.forward(state)
        # rospy.loginfo(" %s ", str(action))
        #noise = self.noise.sample()
        #print('noisea', noise)
        #noise[0] = noise[0]*self.action_limit_v
        #noise[1] = noise[1]*self.action_limit_w
        #print('noise', noise)
        new_action = action.detach().cpu().numpy() #+ noise
        #print('action_no', new_action)
        return new_action
    
    def optimizer(self):
        s_sample, a_sample, r_sample, new_s_sample, done_sample = replay_buffer.sample(BATCH_SIZE)
        
        s_sample = torch.FloatTensor(s_sample).to(self.device)
        a_sample = torch.FloatTensor(a_sample).to(self.device)
        r_sample = torch.FloatTensor(r_sample).to(self.device)
        new_s_sample = torch.FloatTensor(new_s_sample).to(self.device)
        done_sample = torch.FloatTensor(done_sample).to(self.device)
        # rospy.loginfo("s_sample %s , a_sample %s, r_sample %s, new_s_sample %s, done_sample %s", str(s_sample.size()), str(a_sample.size()), str(r_sample.size()), str(new_s_sample.size()), str(done_sample.size()))
        
        #-------------- optimize critic
        
        a_target = self.target_actor.forward(new_s_sample).detach()
        next_value = self.target_critic.forward(new_s_sample, a_target).squeeze(1).detach()
        # rospy.loginfo("next_value %s , new_s_sample %s", str(next_value.size()), str(new_s_sample.size()))
        # y_exp = r _ gamma*Q'(s', P'(s'))
        y_expected = r_sample + (1 - done_sample)*GAMMA*next_value
        # rospy.loginfo("r_sample %s , done_sample %s", str(r_sample.size()), str(done_sample.size()))
        # y_pred = Q(s,a)
        y_predicted = self.critic.forward(s_sample, a_sample).squeeze(1)
        # rospy.loginfo("pred %s , exp %s", str(y_predicted.size()), str(y_expected.size()))
        #-------Publisher of Vs------
        self.qvalue = y_predicted.detach()
        self.pub_qvalue.publish(torch.max(self.qvalue))
        #print(self.qvalue, torch.max(self.qvalue))
        #----------------------------
        # rospy.loginfo("pred %s , exp", str(y_predicted.size()), str(y_expected.size()))
        loss_critic = F.smooth_l1_loss(y_predicted, y_expected)
        
        self.critic_optimizer.zero_grad()
        loss_critic.backward()
        self.critic_optimizer.step()
        
        #------------ optimize actor
        pred_a_sample = self.actor.forward(s_sample)
        loss_actor = -1*torch.sum(self.critic.forward(s_sample, pred_a_sample))
        
        self.actor_optimizer.zero_grad()
        loss_actor.backward()
        self.actor_optimizer.step()
        
        soft_update(self.target_actor, self.actor, TAU)
        soft_update(self.target_critic, self.critic, TAU)
    
    def save_models(self, episode_count):
        torch.save(self.target_actor.state_dict(), dirPath +'/Models/' + world + '/' + str(episode_count)+ '_actor.pt')
        torch.save(self.target_critic.state_dict(), dirPath + '/Models/' + world + '/'+str(episode_count)+ '_critic.pt')
        print('****Models saved***')
        
    def load_models(self, episode):
        self.actor.load_state_dict(torch.load(dirPath + '/Models/' + world + '/'+str(episode)+ '_actor.pt'))
        self.critic.load_state_dict(torch.load(dirPath + '/Models/' + world + '/'+str(episode)+ '_critic.pt'))
        hard_update(self.target_actor, self.actor)
        hard_update(self.target_critic, self.critic)
        print('***Models load***')

def action_unnormalized(action, high, low):
    action = low + (action + 1.0) * 0.5 * (high - low)
    action = np.clip(action, low, high)
    return action

#---Run agent---#

is_training = True

#---Where the train is made---#

BATCH_SIZE = 256
LEARNING_RATE = 3e-4
GAMMA = 0.99
TAU = 0.001

exploration_decay_rate = 0.001

MAX_EPISODES = 10001
MAX_STEPS = 500
MAX_BUFFER = 50000
rewards_all_episodes = []

STATE_DIMENSION = 26
ACTION_DIMENSION = 3
ACTION_V_MAX = 0.25 # m/s
ACTION_V_MIN = 0.0
ACTION_W_MAX = 0.25 # rad
ACTION_W_MIN = -0.25
world = '3bdb'

print('State Dimensions: ' + str(STATE_DIMENSION))
print('Action Dimensions: ' + str(ACTION_DIMENSION))
print('Action Max: ' + str(ACTION_V_MAX) + ' m/s and ' + str(ACTION_W_MAX) + ' rad')
replay_buffer = ReplayBuffer(MAX_BUFFER)
trainer = Trainer(STATE_DIMENSION, ACTION_DIMENSION, ACTION_V_MAX, ACTION_W_MAX, replay_buffer)
# noise = OUNoise(ACTION_DIMENSION, max_sigma=.71, min_sigma=0.2, decay_period=8000000)
noise = OUNoise(ACTION_DIMENSION, max_sigma=.075, min_sigma=0.03, decay_period=8000000)

if __name__ == '__main__':
    global world
    rospy.init_node('ddpg')
    pub_result = rospy.Publisher('result', String, queue_size=5)    
    ep_0 = rospy.get_param('~ep_number')
    world = rospy.get_param('~file_path') 

    if (ep_0 != 0):
        trainer.load_models(ep_0)

    rospy.loginfo("Starting at episode: %s ", str(ep_0))

    result = Float32()
    env = Env(action_dim=ACTION_DIMENSION)
    before_training = 4
    past_action = np.zeros(ACTION_DIMENSION)

    for ep in range(ep_0, MAX_EPISODES):
        done = False
        state = env.reset()
        if is_training and not ep%10 == 0 and len(replay_buffer) >= before_training*BATCH_SIZE:
            rospy.loginfo("---------------------------------")
            rospy.loginfo("Episode: %s training", str(ep))
            rospy.loginfo("---------------------------------")            
        else:
            if len(replay_buffer) >= before_training*BATCH_SIZE:
                rospy.loginfo("---------------------------------")
                rospy.loginfo("Episode: %s evaluating", str(ep))
                rospy.loginfo("---------------------------------")
            else:
                rospy.loginfo("---------------------------------")
                rospy.loginfo("Episode: %s adding to memory", str(ep))
                rospy.loginfo("---------------------------------")

        rewards_current_episode = 0.

        for step in range(MAX_STEPS):
            state = np.float32(state)

            if is_training and not ep%10 == 0:
                action = trainer.get_exploration_action(state)
                
                N = copy.deepcopy(noise.get_noise(t=step))                
                N[0] = N[0]*ACTION_V_MAX/2
                N[1] = N[1]*ACTION_W_MAX
                N[2] = N[2]*ACTION_W_MAX
                # action[0] = action[0] + N[0]
                # action[1] = action[1] + N[1]
                # rospy.loginfo("Noise: %s, %s", str(N[0]), str(N[1]))
                # rospy.loginfo("Action before: %s, %s", str(action[0]), str(action[1]))
                action[0] = np.clip(action[0] + N[0], ACTION_V_MIN, ACTION_V_MAX)
                action[1] = np.clip(action[1] + N[1], ACTION_W_MIN, ACTION_W_MAX)
                action[2] = np.clip(action[2] + N[2], ACTION_W_MIN, ACTION_W_MAX)
            else:
                action = trainer.get_exploration_action(state)

            if not is_training:
                action = trainer.get_exploitation_action(state)
            # unnorm_action = np.array([action_unnormalized(action[0], ACTION_V_MAX, ACTION_V_MIN), action_unnormalized(action[1], ACTION_W_MAX, ACTION_W_MIN)])

            next_state, reward, done = env.step(action, past_action)
            # print('action', action,'r',reward)
            past_action = action

            rewards_current_episode += reward
            next_state = np.float32(next_state)
            if not ep%10 == 0 or not len(replay_buffer) >= before_training*BATCH_SIZE:
                if reward == 100.:
                    rospy.loginfo("--------- Maximum Reward ----------")
                    # print('***\n-------- Maximum Reward ----------\n****')
                    for _ in range(3):
                        replay_buffer.push(state, action, reward, next_state, done)
                else:
                    replay_buffer.push(state, action, reward, next_state, done)

            if len(replay_buffer) > before_training*BATCH_SIZE and is_training and not ep%10 == 0:
                trainer.optimizer()
            state = copy.deepcopy(next_state)   

            if done or step == MAX_STEPS-1:
                rospy.loginfo("Reward per ep: %s", str(rewards_current_episode))
                rospy.loginfo("Break step: %s", str(step))
                rospy.loginfo("Sigma: %s", str(noise.sigma))
                if ep%2 == 0:                   
                    # if ram.len >= before_training*MAX_STEPS:
                    result = (str(ep)+','+str(rewards_current_episode))
                    pub_result.publish(result)
                break

            # if (reward == 100):
            #     is_training = False
            #     break

        if ep%20 == 0:
            trainer.save_models(ep)

print('Completed Training')

# roslaunch hydrone_aerial_underwater_ddpg deep_RL_2D.launch ep:=0 file_dir:=ddpg_stage_1_air2D_tanh_3layers deep_rl:=ddpg_air2D_tanh_3layers.py world:=stage_1_aerial root_dir:=/home/ricardo/

# roslaunch hydrone_aerial_underwater_ddpg deep_RL_2D.launch ep:=380 file_dir:=ddpg_stage_1_air2D_tanh_3layers deep_rl:=ddpg_air2D_tanh_3layers.py world:=stage_1_aerial root_dir:=/home/ricardo/ graphic_int:=false

# roslaunch hydrone_aerial_underwater_ddpg deep_RL_2D.launch ep:=1000 file_dir:=ddpg_stage_1_air3D_tanh_3layers deep_rl:=ddpg_air3D_tanh_3layers.py world:=stage_1 root_dir:=/home/ricardo/ graphic_int:=true testing:=true x:=2.0 y:=3.0 z:=-1.0 arr_distance:=0.25 testing_eps:=200