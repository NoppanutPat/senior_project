#!/usr/bin/env python

import gym
from gym import spaces
from gym.utils import seeding

import numpy as np

import rospy

from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Imu,Image
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray , Float64

import math
import time

import cv2

from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge

def reward_function(x,y):

    sum = 0

    if x > 950 and x < 970:

        sum += 200

    else:

        sum += abs(100/(abs(960-x)**0.5))

    if y > 530 and y < 550:

        sum += 200

    else:

        sum += abs(100/(abs(540-y)**0.5))

    # print("reward : ",sum)

    return  sum

class QuadEnv_point(gym.Env):
    metadata = {'render.modes':['human']}

    def __init__(self):

        super(QuadEnv_point, self).__init__()

        self.high_action = np.array([10,10,10,10],dtype=np.float32)
        self.low_action = -self.high_action

        inf = np.finfo(np.float32).max
        self.inf = inf

        self.high_obs = np.array([inf,inf,inf,inf,inf]) 
        self.low_obs = -self.high_obs

        self.action_space = spaces.Box(low=self.low_action,high=self.high_action,dtype=np.float32)
        self.observation_space = spaces.Box(low=self.low_obs, high=self.high_obs ,dtype=np.float32)

        self.seed()

        self.state = None
        self.viewer = None

        self.reward_store = 0

        self.br = CvBridge()

        rospy.init_node("quadrotor_point_ctl",anonymous=True)

        self.state = np.array([-1000,-1000,-1000,-1000,-1000])
        self.im_data = [-1000,-1000]
        self.imu_data = [-1000,-1000,-1000]


        self.throttle_pub = rospy.Publisher("/quadrotor/throttle",Float64MultiArray,queue_size=10)
        self.reward_pub = rospy.Publisher("/quadrotor/reward",Float64,queue_size=10)

        self.im_pos = rospy.Subscriber("/quadrotor/point_pos",Float64MultiArray,self.update_pos)
        self.imu = rospy.Subscriber("/quadrotor/imu_data",Imu,self.update_imu)

        print("Waiting for /gazebo/reset_simulation topic")
        rospy.wait_for_service("/gazebo/reset_simulation")
        self.reset_sim = rospy.ServiceProxy("/gazebo/reset_simulation",Empty)

        print("Waiting for /quadrotor/point_pos topic")
        rospy.wait_for_message("/quadrotor/point_pos",Float64MultiArray)

        print("Set up success for gazebo message and Goal state message ......")

    def image_callback(self,msg):
        self.image = self.br.imgmsg_to_cv2(msg)

    def update_pos(self,pos):
        self.im_data = [pos.data[0],pos.data[1]]
        self.update_state()

    def update_imu(self,data):
        # print(data.linear_acceleration)
        self.imu_data = [data.linear_acceleration.x,data.linear_acceleration.y,data.linear_acceleration.z]
        self.update_state()

    def update_state(self):
        self.state = np.array(self.im_data+self.imu_data)

    def seed(self , seed=None):
        self.np_random , seed = seeding.np_random(seed)
        return [seed]

    def step(self,action):

        err_msg = "%r (%s) invalid" % (action, type(action))
        assert self.action_space.contains(action), err_msg

        done = False
        duration = time.time() - self.start_time

        x,y,x_ddot,y_ddot,z_ddot = self.state 

        throttle = []

        for i in action:

            throttle.append(min(max(i, -100), 100))

        if len(throttle) != 4:
            print("Throttle error! : Index error")
            return 1

        throttle = Float64MultiArray(data=throttle)
        self.throttle_pub.publish(throttle)

        reward = reward_function(x,y) + 20*(max(0,duration))

        if x == -1000 and y == -1000:

            done=True
            reward = -1000

        elif duration > 120:

            done = True
            reward = 1000
            print("Red dot in frame more than 2 minute.")

        observation = np.array(self.state)

        self.reward_pub.publish(Float64(reward))
            
        return observation, reward, done, {}

    def reset(self):

        throttle = Float64MultiArray(data=[0,0,0,0])

        self.throttle_pub.publish(throttle)

        # self.reward_store = 0

        self.reset_sim()

        self.start_time = time.time()

        observation = np.array(self.state)

        # print("reset : ",observation)

        return observation

    def render(self,mode="human"):

        try:
            scale_percent = 60 # percent of original size
            width = int(self.image.shape[1] * scale_percent / 100)
            height = int(self.image.shape[0] * scale_percent / 100)
            dim = (width, height)
            # resize image
            resized = cv2.resize(self.image, dim, interpolation = cv2.INTER_AREA)

            cv2.imshow("image",resized)
            cv2.waitKey(1)
        
        except Exception:

            # print("Can't find image")
            pass

    def close(self):

        exit(0)

if __name__ == "__main__":
    
    a = QuadEnv_point()
    rospy.spin()