import argparse
import numpy as np

def build_parse():

    parser = argparse.ArgumentParser(description="RL Algorithm Variables")

    parser.add_argument("Environment",nargs="?",type=str,default="hydrone_Circuit_Simple-v0",help="Name of OPEN AI environment")
    parser.add_argument("input_shape",nargs="?",type=int,default=[],help="Shape of environment state")
    parser.add_argument("n_actions",nargs="?",type=int,default=[],help="shape of environment action")
    parser.add_argument("max_action",nargs="?",type=float,default=[],help="Max possible value of action")
    parser.add_argument("min_action",nargs="?",type=float,default=[],help="Min possible value of action")

    parser.add_argument("Algorithm",nargs="?",type=str,default="MADDPG",help="Name of RL algorithm")
    parser.add_argument('tau',nargs="?",type=float,default=0.005)
    parser.add_argument('gamma',nargs="?",default=0.99)
    parser.add_argument('actor_lr',nargs="?",type=float,default=0.0001,help="Learning rate of Policy Network")
    parser.add_argument('critic_lr',nargs="?",type=float,default=0.0001,help="Learning rate of the Q Network")
    parser.add_argument('is_continous',nargs="?",type=bool,default=True,help="Action space is discrete or continous")

    parser.add_argument("mem_size",nargs="?",type=int,default=1000000,help="Size of Replay Buffer")
    parser.add_argument("batch_size",nargs="?",type=int,default=64,help="Batch Size used during training")
    parser.add_argument("n_agents",nargs="?",type=int,default=2,help="Total number of agents in the environment")
    parser.add_argument("n_episodes",nargs="?",type=int,default=100000,help="Total number of episodes to train the agent")
    parser.add_argument("n_batches",nargs="?",type=int,default=10,help="Total number of times the RL needs to be replicated")
    parser.add_argument("target_update",nargs="?",type=int,default=10,help="Iterations to update the target network")
    parser.add_argument("vision_update",nargs="?",type=int,default=5,help="Iterations to update the vision network")
    parser.add_argument("delayed_update",nargs="?",type=int,default=100,help="Iterations to update the second target network using delayed method")

    # SOFT ACTOR PARAMETERS
    parser.add_argument("temperature",nargs="?",type=float,default=0.2,help="Entropy Parameter")
    parser.add_argument("log_std_min",nargs="?",type=float,default=np.log(1e-4),help="")
    parser.add_argument("log_std_max",nargs="?",type=float,default=np.log(4),help="")
    parser.add_argument("aux_step",nargs="?",type=int,default=8,help="How often the auxiliary update is performed")
    parser.add_argument("aux_epoch",nargs="?",type=int,default=6,help="How often the auxiliary update is performed")
    parser.add_argument("target_entropy_beta",nargs="?",type=float,default=-3,help="")
    parser.add_argument("target_entropy",nargs="?",type=float,default=-3,help="")

    # MISC VARIABLES
    parser.add_argument("save_rl_weights",nargs="?",type=bool,default=True,help="save reinforcement learning weights")
    parser.add_argument("save_results",nargs="?",type=bool,default=True,help="Save average rewards using pickle")

    args = parser.parse_args("")

    return args

def get_maddpg_args(args):

    args.is_continous = True

    args.critic_hidden = 64
    args.policy_hidden = 64

    args.critic_lr = 0.001
    args.actor_lr = 0.001

    args.target_update = 1
    args.batch_size = 1024

    args.tau = 0.01
    args.gamma = 0.95

    return args

def get_facmac_args(args):

    args.is_continous = True

    args.critic_hidden = 64
    args.policy_hidden = 64
    args.mixer_hidden = 64

    args.target_update = 2

    return args

def get_vdn_args(args):

    args.is_continous = True

    args.critic_hidden = 64
    args.policy_hidden = 64

    args.target_update = 2

    args.epsilon = 1.0
    args.epsilon_min = 0.05

    return args

def get_coma_args(args):

    args.is_continous = False

    args.critic_hidden = 64
    args.policy_hidden = 64
    args.target_update = 150
    args.epsilon = 1.0
    args.epsilon_min = 0.05

    args.train_network = int(30/args.n_agents)

    args.grad_norm_clip = 10

    args.critic_lr = 0.01
    args.actor_lr = 0.01

    args.gamma = 0.95
    args.batch_size = 128

    return args

def get_qmix_args(args):

    args.is_continous = False

    args.rnn_hidden = 64
    args.policy_hidden = 64
    args.mixer_hidden = 64

    args.grad_norm_clip = 10

    args.epsilon = 1.0
    args.epsilon_min = 0.05

    return args

def get_env_parameters(args,envs):

    args.state_size = {}
    args.input_shape = {}
    args.n_actions = {}
    args.action_space = {}
    # all agents have the same observation_size and shape
    args.state_shape = envs[0].observation_space.shape
    args.max_action = {}
    args.min_action = {}
    args.env_agents = envs
    args.n_agents = len(envs)

    for i in range(0, len(envs)):
        if args.is_continous:
            args.state_size[i] = envs[i].observation_space.shape[0]
            args.input_shape[i] = envs[i].observation_space.shape[0]
            args.n_actions[i] = envs[i].action_space.shape[0]
            args.max_action[i] = envs[i].action_space.high
            args.min_action[i] = envs[i].action_space.low
            args.action_space[i] = args.n_actions[i]
        # else:
        #     args.state_size[i] = env.observation_space(agent).shape[0]
        #     args.input_shape[i] = env.observation_space(agent).shape[0]
        #     args.n_actions[i] = env.action_space(agent).n
        #     args.action_space[i] = 1

    return args
