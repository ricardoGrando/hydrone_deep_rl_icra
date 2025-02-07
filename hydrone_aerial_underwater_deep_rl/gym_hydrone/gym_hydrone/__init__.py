from gym.envs.registration import register

register(
    id='hydrone-v0',
    entry_point='gym_hydrone.envs:hydroneEnv'
)

goal_list = [   [0.505, 2.005, 1.0],
                [1.005, 2.505, 1.0],
                [2.005, 2.505, 1.0],
                [2.505, 2.005, 1.0],
                [2.505, 1.005, 1.0],
                [2.005, 0.505, 1.0],
                [1.005, 0.505, 1.0],
                [0.505, 1.005, 1.0]]

max_env_size = 3

register(
    id='hydrone_Circuit_Simple-v0',
    entry_point='gym_hydrone.envs:hydroneEnv',
    kwargs={'goal_list': goal_list, 'max_env_size': max_env_size}
)

register(
    id='hydrone_Circuit_Simple_Continuous-v0',
    entry_point='gym_hydrone.envs:hydroneEnv',
    kwargs={'goal_list': goal_list, 'max_env_size': max_env_size, 'continuous': True}
)

goal_list = [   [0.505, 4.005, 1.0],
                [1.005, 4.505, 1.0],
                [2.005, 4.505, 1.0],
                [2.505, 4.005, 1.0],
                [2.505, 3.005, 1.0],
                [3.005, 2.505, 1.0],
                [4.005, 2.505, 1.0],
                [4.505, 2.005, 1.0],
                [4.505, 1.005, 1.0],
                [4.005, 0.505, 1.0],
                [1.005, 0.505, 1.0],
                [0.505, 1.005, 1.0],
                [0.505, 2.5, 1.0]]

max_env_size = 5

register(
    id='hydrone_Circuit_Left_Right_Turns-v0',
    entry_point='gym_hydrone_fuzzy.envs:hydroneFuzzyEnv',
    kwargs={'goal_list': goal_list, 'max_env_size': max_env_size}
)
