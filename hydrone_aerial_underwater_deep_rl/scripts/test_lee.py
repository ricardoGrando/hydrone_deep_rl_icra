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
from std_srvs.srv import Empty
from datetime import datetime
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range

pub = rospy.Publisher('/hydrone_aerial_underwater/command/pose', PoseStamped, queue_size=10)

pub_cmd_vel = rospy.Publisher('/hydrone_aerial_underwater/cmd_vel', Twist, queue_size=5)        
pub_end = rospy.Publisher("/hydrone_aerial_underwater/end_testing", Bool, queue_size=5)
eps_to_test = 100
counter_eps = 0
last_time = datetime.now() 
pub_reward = rospy.Publisher("/hydrone_aerial_underwater/rewarded", Bool, queue_size=5)

# posx = [0.0, 2.0, 0.0, -2.0, -2.0, 0.0, 2.0, 0.0]
# posy = [0.0, 2.0, 3.0, 2.0, -2.0, -3.0, -2.0, 0.0]
# posz = [-1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5]

# i = 0

# posx = [0.0, 3.6, -3.6, -3.6, 0.0]
# posy = [0.0, 2.6, 3.0, 1.0, 0.0]
# posz = [-1.5, -1.5, -1.5, -1.5, -1.5]

# posx = [0.0, -1.5, 0.0, -1.5, -1.5, 0.0, 1.5, 0.0]
# posy = [0.0, -1.5, 1.5, 1.5, -1.5, -1.5, -1.5, 0.0]
# posz = [2.5, 2.5, 1.0, 2.5, 1.0, 2.5, 1.0, 2.5]

# posx = [0.0, 3.6, -3.6, -3.6, 0.0]
# posy = [0.0, 2.6, 3.0, 1.0, 0.0]
# posz = [2.5, 1.0, 2.5, 1.0, 2.5]

# posx = [0.0]
# posy = [0.0]
# posz = [2.5]

posx = [2.0]
posy = [3.0]
posz = [-1.0]

# goal_x_list = [3.6, -3.6, -3.6, 0.0]
# goal_y_list = [2.6, 3.0, 1.0, 0.0]
   
# goal_x_list = [2.0, 0.0, -2.0, -2.0, 0.0, 2.0, 0.0]
# goal_y_list = [2.0, 3.0, 2.0, -2.0, -3.0, -2.0, 0.0]

# def state_callback(data):
#     global i

#     i = data.data

#     print(i)

_data = Odometry()
scan = LaserScan()

def position_callback(data):
    global _data
    _data = data

def laser_callback(data):
    global scan
    scan = data

if __name__ == "__main__":   
    global posx
    global posy
    global posz
    global pub
    global pub_cmd_vel
    global last_time
    global counter_eps
    global eps_to_test
    global pub_end
    global pub_reward
    global _data
    global scan

    rospy.init_node("test_lee", anonymous=False)    

    rospy.Subscriber("/hydrone_aerial_underwater/ground_truth/odometry", Odometry, position_callback)

    rospy.Subscriber("/hydrone_aerial_underwater/scan", LaserScan, laser_callback)

    # rospy.Subscriber("/hydrone_aerial_underwater/next_position_pid", Int64, state_callback)
   
    while not rospy.is_shutdown():      

        distance = math.sqrt((posx[0] - _data.pose.pose.position.x)**2 + (posy[0] - _data.pose.pose.position.y)**2 + (posz[0] - _data.pose.pose.position.z)**2)
        
        pose = PoseStamped()
        pose.pose.position.x = posx[0]
        pose.pose.position.y = posy[0]
        pose.pose.position.z = posz[0]

        # scan = rospy.wait_for_message('/hydrone_aerial_underwater/scan', LaserScan, timeout=5)
        # rospy.loginfo(str(min(scan.ranges)))

        while len(scan.ranges) == 0:
            rospy.loginfo("Waiting for laser")

        if (distance < 0.25 or min(scan.ranges) < 0.6):
            # i += 1
            # print(i)

            rospy.wait_for_service('gazebo/reset_simulation')
            try:
                reset_proxy = rospy.ServiceProxy('gazebo/reset_world', Empty)
                reset_proxy()
            except (rospy.ServiceException) as e:
                print("gazebo/reset_simulation service call failed")

            timer = Twist()
            timer.linear.y = (datetime.now() - last_time).total_seconds()
            pub_cmd_vel.publish(timer)
            last_time = datetime.now()

            if (distance < 0.25):
                pub_reward.publish(True)
            
            counter_eps += 1

            if(counter_eps == eps_to_test):
                pub_end.publish(False)
                rospy.signal_shutdown("end_test")

            time.sleep(2)

        pub.publish(pose)

# roslaunch hydrone_aerial_underwater_ddpg lee_controller.launch root_dir:=/home/ricardo/ file_dir:=lee_stage_1 testing:=true world:=stage_1

    