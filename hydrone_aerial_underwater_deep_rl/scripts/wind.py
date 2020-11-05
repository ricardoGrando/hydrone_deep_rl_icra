#! /usr/bin/env python
import numpy as np
from rotors_comm.msg import *
import rospy

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



if __name__ == "__main__":     
    rospy.init_node("wind_node", anonymous=False)    

    noise = OUNoise(2, max_sigma=.075, min_sigma=0.03, decay_period=8000000)

    pub = rospy.Publisher('/hydrone_aerial_underwater/wind_speed', WindSpeed, queue_size=10)    
    r = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        speed = WindSpeed()
        n = noise.get_noise()
        speed.velocity.x = n[0]
        speed.velocity.y = n[1]
        # speed.velocity.z = random.choice([random.uniform(-4.0, -3.0), random.uniform(3.0, 4.0)])
        pub.publish(speed)
        r.sleep()  

        # rospy.loginfo("Wind: %s, %s ", str(n[0]), str(n[1]))      
