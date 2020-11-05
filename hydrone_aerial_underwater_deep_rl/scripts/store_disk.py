#! /usr/bin/env python
import time
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *

PATH = 'sac_stage_1/sac_env1_2d_3_layers'
ROOT = '/home/ricardo/'
TESTING = False

def store_disk(data):
    global PATH
    global ROOT
    file_object = open(ROOT+'catkin_ws/src/hydrone_aerial_underwater_gazebo/hydrone_aerial_underwater_ddpg/scripts/Models/'+PATH+'/'+PATH+'.csv', 'a')
    
    file_object.write(data.data+'\n')

def pose_callback(data):   
    # print(data.position.x)
    file_object = open(ROOT+'catkin_ws/src/hydrone_aerial_underwater_gazebo/hydrone_aerial_underwater_ddpg/scripts/position_'+PATH+'.csv', 'a')
    file_object.write(str(data.position.x)+","+str(data.position.y)+","+str(data.position.z)+'\n')
    # time.sleep(0.1)
    # print(data)

def cmd_callback(data):    
    # print(data.position.x)
    file_object = open(ROOT+'catkin_ws/src/hydrone_aerial_underwater_gazebo/hydrone_aerial_underwater_ddpg/scripts/cmd_'+PATH+'.csv', 'a')
    file_object.write(str(data.linear.x)+","+str(data.linear.y)+","+str(data.linear.z)+","+str(data.angular.z)+'\n')
    # time.sleep(0.1)
    # print(data)

def rewarded_callback(data):    
    # print(data.position.x)
    file_object = open(ROOT+'catkin_ws/src/hydrone_aerial_underwater_gazebo/hydrone_aerial_underwater_ddpg/scripts/rewarded_'+PATH+'.csv', 'a')
    file_object.write(str(data.data)+'\n')
    # time.sleep(0.1)
    # print(data)

def end_callback(data):
    rospy.signal_shutdown("end_test")

if __name__ == "__main__":
    global PATH 
    global ROOT
    global TESTING
    rospy.init_node("store_disk", anonymous=False)   

    PATH = rospy.get_param('~file_path') 
    ROOT = rospy.get_param('~root_path') 
    TESTING = rospy.get_param('~test_param')      

    if (TESTING):        
        rospy.Subscriber("/hydrone_aerial_underwater/ground_truth/pose", Pose, pose_callback)
        rospy.Subscriber("/hydrone_aerial_underwater/cmd_vel", Twist, cmd_callback)
        rospy.Subscriber("/hydrone_aerial_underwater/end_testing", Bool, end_callback)
        rospy.Subscriber("/hydrone_aerial_underwater/rewarded", Bool, rewarded_callback)
    else:
        rospy.Subscriber("/result", String, store_disk)

    rospy.spin()

# roslaunch hydrone_aerial_underwater_ddpg deep_RL_2D.launch ep:=1000 file_dir:=ddpg_stage_1_air2D_tanh_3layers deep_rl:=ddpg_air2D_tanh_3layers.py world:=stage_1_aerial root_dir:=/home/ricardo/ graphic_int:=false testing:=true x:=2.0 y:=2.0 arr_distance:=0.1 testing_eps:=5
