#! /usr/bin/env python
import time
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geographic_msgs.msg import *
from trajectory_msgs.msg import *
from nav_msgs.msg import Odometry
import math

trans = Transform()
cmd_vel = Twist()

cmd_vel_x = 0

pub = rospy.Publisher('/hydrone_aerial_underwater/command/trajectory', MultiDOFJointTrajectory, queue_size=10)

def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]

def velocity_callback(data):
    global cmd_vel
    global cmd_vel_x

    cmd_vel = data

    cmd_vel_x = cmd_vel.linear.x

def position_callback(data):
    #print(data.pose.pose.position.x)
    trans.translation.x = data.pose.pose.position.x
    trans.translation.y = data.pose.pose.position.y
    trans.translation.z = data.pose.pose.position.z
    trans.rotation.x = data.pose.pose.orientation.x
    trans.rotation.y = data.pose.pose.orientation.y
    trans.rotation.z = data.pose.pose.orientation.z
    trans.rotation.w = data.pose.pose.orientation.w

    # cmd_vel.linear.z = 0.1
    #cmd_vel.linear.x = 0.0

    #cmd_vel.angular.z = 0.5

    yaw, pitch, roll = quaternion_to_euler(trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w)

    # # print(yaw)
    rotation = yaw + cmd_vel.angular.z

    # # print(rotation)

    cmd_vel.linear.x = cmd_vel_x*math.cos(rotation)
    cmd_vel.linear.y = cmd_vel_x*math.sin(rotation) 
    # cmd_vel.linear.x = cmd_vel_x*math.cos(rotation)
    
    # cmd_vel.linear.z = -2.143
    # cmd_vel.linear.z = -0.02
    
    # print("X: "+str(cmd_vel.linear.x))
    # print("Y: "+str(cmd_vel.linear.y))

    point = MultiDOFJointTrajectoryPoint()
    velocity = MultiDOFJointTrajectory()

    point.transforms.append(trans)
    point.velocities.append(cmd_vel)
    velocity.points.append(point)

    pub.publish(velocity)    

if __name__ == "__main__": 
    rospy.init_node("mission_planner_node", anonymous=False)    

    rospy.Subscriber("/hydrone_aerial_underwater/ground_truth/odometry", Odometry, position_callback)
    rospy.Subscriber("/hydrone_aerial_underwater/cmd_vel", Twist, velocity_callback)

    rospy.spin()