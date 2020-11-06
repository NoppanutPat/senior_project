import gym
from gym import spaces
from gym.utils import seeding

import numpy as np

import message_filters
import rospy

from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Imu,Image
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray , Float64

import math
import time

import cv2

import sys
sys.path = ["/home/pat/catkin_ws/devel/lib/python3/dist-packages"] + ["/home/pat/catkin_workspace/devel/lib/python3/dist-packages"] + sys.path

from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge

class QuadEnv(gym.Env):
    metadata = {'render.modes':['human']}

    def __init__(self):

        super(QuadEnv, self).__init__()

        self.low_action = np.array([-100,-100,-100,-100],dtype=np.float32)
        self.high_action = np.array([100,100,100,100],dtype=np.float32)

        inf = np.finfo(np.float32).max
        self.inf = inf

        # Observation R,P,Y,R_dot,P_dot,Y_dot,x_ddot,y_ddot,z_ddot

        self.high_obs = np.array([inf,inf,inf,inf,inf,inf,inf,inf,inf]) 
        self.low_obs = -self.high_obs

        self.action_space = spaces.Box(low=self.low_action,high=self.high_action,dtype=np.float32)
        self.observation_space = spaces.Box(low=self.low_obs, high=self.high_obs ,dtype=np.float32)

        self.seed()

        self.state = None
        self.viewer = None

        self.step_before = 0

        self.br = CvBridge()

        rospy.init_node("quadrotor",anonymous=True)

        self.throttle_pub = rospy.Publisher("/quad_sim/throttle",Float64MultiArray,queue_size=10)
        self.reward_pub = rospy.Publisher("/quadrotor/reward",Float64,queue_size=10)

        self.obs_pos = message_filters.Subscriber("/gazebo/model_states",ModelStates)
        self.obs_imu = message_filters.Subscriber("/imu_data",Imu)
        self.goal_state = message_filters.Subscriber("/goal_state",Float64MultiArray) # [x,y,z]

        self.image_sub = rospy.Subscriber("/camera1/image_raw",Image,self.image_callback)

        self.ms_fil = message_filters.ApproximateTimeSynchronizer([self.obs_pos,self.obs_imu,self.goal_state],10,0.5,allow_headerless=True)
        self.ms_fil.registerCallback(self.callback)

        print("Waiting for Goal state message.....",end="\r")
        rospy.wait_for_message("/goal_state",Float64MultiArray)

        
        print("Waiting for gazebo message......",end="\r")        
        rospy.wait_for_message("/gazebo/model_states",ModelStates)
        rospy.wait_for_message("/imu_data",Imu)

        sys.stdout.flush()

        print("Set up success for gazebo message and Goal state message ......")

    def image_callback(self,msg):

        self.image = self.br.imgmsg_to_cv2(msg)

    def callback(self,pos,imu,goal):

        index_pos = pos.name.index("quadrotor")
        pose = pos.pose[index_pos].position
        twist = pos.twist[index_pos].linear
        q_orientation = [imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w]

        tmp = euler_from_quaternion(q_orientation)
        self.orientation = Vector3()
        self.orientation.x = tmp[0]
        self.orientation.y = tmp[1]
        self.orientation.z = tmp[2]
        self.angualar_velo = imu.angular_velocity
        self.linear_acc = imu.linear_acceleration
        self.pose = pose
        self.twist = twist
        self.goal_h = goal.data[2]

        self.current_time = time.time()

        self.state = [float(self.pose.x),float(self.pose.y),float(self.pose.z),float(self.orientation.x),float(self.orientation.y),float(self.orientation.z),float(self.angualar_velo.x),float(self.angualar_velo.y),float(self.angualar_velo.z)]

        # print(self.goal_h)

    def seed(self , seed=None):
        self.np_random , seed = seeding.np_random(seed)
        return [seed]

    def step(self,action):

        err_msg = "%r (%s) invalid" % (action, type(action))
        assert self.action_space.contains(action), err_msg

        done = False

        try:

            x,y,z,x_dot,y_dot,z_dot,x_ddot,y_ddot,z_ddot = self.state 

        except Exception :

            return 0,0,0,0

        throttle = []

        for i in action:

            throttle.append(min(max(i, -100), 100))

        if len(throttle) != 4:
            print("Throttle error! : Index error")
            return 1

        throttle = Float64MultiArray(data=throttle)
        # print(throttle)
        self.throttle_pub.publish(throttle)
        rospy.Rate(10)

        if abs(self.pose.x) <= 1 and abs(self.pose.y) <= 1 and abs(self.goal_h - self.pose.z) <= 1E-02 :

            self.step_before += 1

        def normal_curve(x,sd=1.0):

            return float(1/(sd*((2*math.pi)**0.5)))*(math.e**(-(x**2)/(2*sd*sd)))

        delta_h = (self.goal_h - self.pose.z) + (10*self.orientation.x) + (10*self.orientation.y) + (10*self.orientation.z)

        reward = 100*normal_curve(delta_h) + (self.step_before**2)

        print(reward)

        # print(normal_curve(delta_h))

        if self.pose.z > self.goal_h*3 or abs(self.pose.x) >= 0.5 or abs(self.pose.y) >= 0.5 or abs(self.orientation.x) >= 0.5 or abs(self.orientation.y) >= 0.5:

            done=True
            reward = -100
        
        dif_time = time.time() - self.current_time
        if dif_time > 5000:
            done=True
            if self.step_before > 10:
                reward = 100
            else:
                reward = -100

        # if abs(delta_h) <= 1E-04:
            
        #     reward = 100

        observation = np.array(self.state)

        self.reward_pub.publish(Float64(reward))
            
        return observation, reward, done, {}

    def reset(self):

        throttle = Float64MultiArray(data=[0,0,0,0])

        self.throttle_pub.publish(throttle)

        rospy.wait_for_service("/gazebo/reset_simulation")

        reset_sim = rospy.ServiceProxy("/gazebo/reset_simulation",Empty)

        reset_sim()

        observation = np.array(self.state)

        self.step_before = 0

        # print("reset")

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
    
    a = QuadEnv()
    rospy.spin()