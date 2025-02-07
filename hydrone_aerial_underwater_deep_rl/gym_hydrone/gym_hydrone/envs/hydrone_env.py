#! /usr/bin/env python3

import rospy
import gym
import numpy as np
import math
import time
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from gym import spaces
from gym.utils import seeding
from gym_hydrone.envs.mytf import euler_from_quaternion
from gym_hydrone.envs import Respawn

class hydroneEnv(gym.Env):
    def __init__(self, observation_mode=0, env_stage=1, max_env_size=None, continuous=False, observation_size=24,
                 action_size=5, min_range=0.5, max_range=10, min_ang_vel=-0.25, max_ang_vel=0.25, min_linear_vel=-0.25,
                 max_linear_vel=0.25, min_altitude_vel=-0.25, max_altitude_vel=0.25, goalbox_distance=0.85, collision_distance=0.65, reward_goal=200.,

                 reward_collision=-20, angle_out=250, goal_list=None, test_real=False, agent_number=0, model_path='/home/ricardo/hydrone_ws/src/hydrone_deep_rl_icra/hydrone_aerial_underwater_deep_rl/models/goal_box/model.sdf'):

        self.goal_x = 0
        self.goal_y = 0
        self.goal_z = 0
        self.heading = np.array([0., 0.])
        self.image = None
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.env_stage = env_stage
        self.test_real = test_real
        self.agent_number = agent_number

        self.pub_cmd_vel = rospy.Publisher('/hydrone_aerial_underwater'+str(self.agent_number)+'/cmd_vel', Twist, queue_size=1)
        self.sub_odom = rospy.Subscriber('/hydrone_aerial_underwater'+str(self.agent_number)+'/odometry_sensor1/odometry', Odometry, self.getOdometry)
        
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.respawn_goal = Respawn(model_path)

        if not goal_list:
            # if self.env_stage == 1 or self.env_stage == 2:
            goal_list = np.asarray([np.random.uniform((-3.5, -3.5, -0.9), (3.5, 3.5, 3.5)) for _ in range(1)])
            # else:
            #     goal_list = np.array([np.random.uniform((0, 0), (3, -3)) for _ in range(1)])
        else:
            goal_list = np.array(goal_list)
        self.respawn_goal.setGoalList(goal_list)

        self.observation_mode = observation_mode
        self.observation_size = observation_size
        self.min_range = min_range
        self.max_range = max_range
        self.min_ang_vel = min_ang_vel
        self.max_ang_vel = max_ang_vel
        self.min_linear_vel = min_linear_vel
        self.max_linear_vel = max_linear_vel
        self.min_altitude_vel = min_altitude_vel
        self.max_altitude_vel = max_altitude_vel
        self.goalbox_distance = goalbox_distance
        self.collision_distance = collision_distance
        self.reward_goal = reward_goal
        self.reward_collision = reward_collision
        self.angle_out = angle_out
        self.continuous = continuous
        self.max_env_size = max_env_size

        if self.continuous:
            low, high, shape_value = self.get_action_space_values()
            self.action_space = spaces.Box(low=low, high=high, shape=(shape_value,))
        else:
            self.action_space = spaces.Discrete(action_size)
            ang_step = max_ang_vel / ((action_size - 1) / 2)
            self.actions = [((action_size - 1) / 2 - action) * ang_step for action in range(action_size)]

        low, high = self.get_observation_space_values()
        self.observation_space = spaces.Box(low, high)

        self.num_timesteps = 0
        self.lidar_distances = None
        self.ang_vel = 0
        self.linear_vel = 0

        self.start_time = time.time()
        self.last_step_time = self.start_time

        self.seed()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def get_action_space_values(self):
        low = np.array([self.min_ang_vel, self.min_linear_vel, self.min_altitude_vel])
        high = np.array([self.max_ang_vel, self.max_linear_vel, self.max_altitude_vel])
        shape_value = 3
        return low, high, shape_value

    def get_observation_space_values(self):
        low = np.append(np.full(self.observation_size, self.min_range), np.array([-math.pi, 0], dtype=np.float32))
        high = np.append(np.full(self.observation_size, self.max_range), np.array([math.pi, self.max_env_size], dtype=np.float32))
        return low, high

    def _getGoalDistace(self):
        goal_distance = math.sqrt((self.goal_x - self.position.x)**2 + (self.goal_y - self.position.y)**2 + (self.goal_z - self.position.z)**2)
        return goal_distance

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        yaw_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)
        pitch_angle = math.atan2(self.goal_z - self.position.z, self.goal_x - self.position.x)

        heading = np.array([0., 0.])
        heading[0] = yaw_angle - yaw
        heading[1] = pitch_angle - pitch
        for i in range(2):
            if heading[i] > math.pi:
                heading[i] -= 2 * math.pi

            elif heading[i] < -math.pi:
                heading[i] += 2 * math.pi

        self.heading = heading

    def get_time_info(self):
        time_info = time.strftime("%H:%M:%S", time.gmtime(time.time() - self.start_time))
        time_info += '-' + str(self.num_timesteps)
        return time_info

    def episode_finished(self):
        pass

    def get_env_state(self):
        return self.lidar_distances

    def getState(self, scan):
        scan_range = []
        heading = self.heading
        done = False

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(self.max_range)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(self.min_range)
            else:
                scan_range.append(scan.ranges[i])

        self.lidar_distances = scan_range

        if self.test_real:
            while self.image is None:
                time.sleep(0.1)
            return [self.get_env_state(), self.image]

        time_info = self.get_time_info()
        current_distance = self._getGoalDistace()
        if min(self.lidar_distances) < self.collision_distance:
            # print(f'{time_info}: Collision!!')
            done = True

        if (self.position.z < -0.9 or self.position.z > 5.0):
            done = True

        if current_distance < self.goalbox_distance:
            if not done:
                # print(f'{time_info}: Goal!!')
                self.get_goalbox = True
                if self.respawn_goal.last_index is (self.respawn_goal.len_goal_list - 1):
                    done = True
                    self.episode_finished()
        return self.get_env_state() + [heading[0], heading[1], current_distance], done

    def get_done_reward(self, lidar, distance):
        done = False
        self.lidar_distances = lidar
        if min(self.lidar_distances) < self.collision_distance:
            done = True

        if (self.position.z < -0.9 or self.position.z > 4.0):
            done = True

        if distance < self.goalbox_distance:
            if not done:
                self.get_goalbox = True
                if self.respawn_goal.last_index is (self.respawn_goal.len_goal_list - 1):
                    done = True

        reward = self.setReward(done)
        return reward, done

    def setReward(self, done):
        if self.get_goalbox:
            reward = self.reward_goal
            self.pub_cmd_vel.publish(Twist())
            self.goal_x, self.goal_y, self.goal_z = self.respawn_goal.getPosition(True, delete=True)
            self.goal_distance = self._getGoalDistace()
            self.get_goalbox = False
        elif done:
            reward = self.reward_collision =- 20.
            self.pub_cmd_vel.publish(Twist())
            if self.respawn_goal.last_index != 0:
                self.respawn_goal.initIndex()
                self.goal_x, self.goal_y, self.goal_z= self.respawn_goal.getPosition(True, delete=True)
                self.goal_distance = self._getGoalDistace()
        else:
            reward = 0
        return reward

    def set_ang_vel(self, action):
        if self.continuous:
            self.ang_vel = action
        else:
            self.ang_vel = self.actions[action]

    def set_linear_vel(self, action):
        if self.continuous:
            self.linear_vel = action
        else:
            self.linear_vel = self.actions[action]

    def set_altitude_vel(self, action):
        if self.continuous:
            self.altitude_vel = action
        else:
            self.altitude_vel = self.actions[action]

    def step(self, action):
        self.set_linear_vel(np.clip(action[0], self.min_linear_vel, self.max_linear_vel))
        self.set_altitude_vel(np.clip(action[1], self.min_altitude_vel, self.max_altitude_vel))
        self.set_ang_vel(np.clip(action[2], self.min_ang_vel, self.max_ang_vel))

        vel_cmd = Twist()
        vel_cmd.linear.x = self.linear_vel
        vel_cmd.linear.z = self.altitude_vel
        vel_cmd.angular.z = self.ang_vel
        self.pub_cmd_vel.publish(vel_cmd)

        data = None
        while data is None:
            try:
                if self.test_real:
                    data = rospy.wait_for_message('/hydrone_aerial_underwater'+str(self.agent_number)+'/scan', LaserScan, timeout=5)
                else:
                    data = rospy.wait_for_message('/hydrone_aerial_underwater'+str(self.agent_number)+'/scan', LaserScan, timeout=5)
            except Exception:
                pass

        self.num_timesteps += 1
        if self.test_real:
            state = self.getState(data)
            return state, None, None, {}
        else:
            state, done = self.getState(data)
            reward = self.setReward(done)
            return np.asarray(state), reward, done, {}

    def get_position(self):
        return [self.position.x, self.position.y, self.position.z]

    def get_scan(self):
        return self.lidar_distances

    def reset(self, new_random_goals=True, goal=None):
        if not self.test_real:
            if new_random_goals:
                if self.env_stage == 1:
                    self.respawn_goal.setGoalList(np.asarray([np.random.uniform((-3.5, -3.5, -0.9), (3.5, 3.5, 3.5)) for _ in range(1)]))
                if self.env_stage == 2:
                    targets = np.asarray([np.random.uniform((-3.5, -3.5, -0.9), (3.5, 3.5, 3.5)) for _ in range(1)])
                    if ((targets[0,0] < -1.6 and targets[0,0] > -2.9) or (targets[0,0] > 1.6 and targets[0,0] < 2.9)
                        and (targets[0,1] < -1.6 and targets[0,1] > -2.9) or (targets[0,1] > 1.6 and targets[0,1] < 2.9)):
                        targets = np.asarray([np.random.uniform((-3.5, -3.5, -0.9), (3.5, 3.5, 3.5)) for _ in range(1)])
                    self.respawn_goal.setGoalList(targets)
                if self.env_stage == 3:
                    targets = np.asarray([np.random.uniform((-4.0, -4.0, -0.9), (4.0, 4.0, 3.5)) for _ in range(1)])
                    if not(((targets[0,0] > -3.15 and targets[0,0] < -4.0) and (targets[0,1] > 2.25 and targets[0,1] < -4.3)) or
                           ((targets[0,0] > 4.3 and targets[0,0] < 0.6) and (targets[0,1] > -2.25 and targets[0,1] > -4.3)) or
                           ((targets[0,0] > 4.3 and targets[0,0] < 3.6) and (targets[0,1] > 4.3 and targets[0,1] < 4.1)) or
                           ((targets[0,0] > 1.6 and targets[0,0] < 0.0) and (targets[0,1] > 1.5 and targets[0,1] < -4.3)) or
                           ((targets[0,0] > 1.3 and targets[0,0] < -1.44) and (targets[0,1] > 4.3 and targets[0,1] < 3.4))):
                        targets = np.asarray([np.random.uniform((-4.0, -4.0, -0.9), (4.0, 4.0, 3.5)) for _ in range(1)])
                    self.respawn_goal.setGoalList(targets)
                else:
                    self.respawn_goal.setGoalList(np.asarray([np.random.uniform((-3.5, -3.5, -0.9), (3.5, 3.5, 3.5)) for _ in range(1)]))
            else:
                self.respawn_goal.setGoalList(np.array(goal))

            rospy.wait_for_service('/gazebo/reset_world')
            try:
                self.reset_proxy()
            except rospy.ServiceException:
                print("gazebo/reset_simulation service call failed")

            data = None
            while data is None:
                try:
                    data = rospy.wait_for_message('/hydrone_aerial_underwater'+str(self.agent_number)+'/scan', LaserScan, timeout=5)
                except:
                    pass

            if self.initGoal:
                self.goal_x, self.goal_y, self.goal_z = self.respawn_goal.getPosition()
                self.initGoal = False
                time.sleep(1)
            else:
                self.goal_x, self.goal_y, self.goal_z = self.respawn_goal.getPosition(True, delete=True)

            self.goal_distance = self.old_distance = self._getGoalDistace()
            state, _ = self.getState(data)
        else:
            data = None
            while data is None:
                try:
                    data = rospy.wait_for_message('/hydrone_aerial_underwater'+str(self.agent_number)+'/scan', LaserScan, timeout=5)
                except:
                    pass
            state = self.getState(data)

        return state

    def render(self, mode=True):
        pass

    def close(self):
        self.reset()
