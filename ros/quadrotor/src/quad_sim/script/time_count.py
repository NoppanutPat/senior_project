#!/usr/bin/env python3

import rospy
import message_filters

import numpy as np
import time

from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray , Float64

class time_count(object):

    def __init__(self):

        rospy.init_node("time_count",anonymous=True)

        pos = message_filters.Subscriber("/gazebo/model_states",ModelStates)
        goal_state = message_filters.Subscriber("/goal_state",Float64MultiArray)

        ms_fil = message_filters.ApproximateTimeSynchronizer([pos,goal_state],10,0.5,allow_headerless=True)

        ms_fil.registerCallback(self.callback)

        self.pub = rospy.Publisher("/time_count/time",Float64,queue_size=10)

        self.on_pos_x = False
        self.on_pos_y = False
        self.on_pos_z = False

        self.start_time = None

        self.elips = 0

    def callback(self,pos,goal_state):

        index_pos = pos.name.index("quadrotor")
        pose = pos.pose[index_pos].position
        pose = np.array([pose.x,pose.y,pose.z])
        goal = np.array(goal_state.data)

        self.dif = abs(goal - pose)

        print("pose : ",pose)
        print("goal : ",goal)

        self.time_count()

    def time_count(self):

        print((self.dif <= 0.1).all())

        if (self.dif <= 0.1).all():

            if self.start_time is None:

                self.start_time = time.time()
                self.elips = 0
        
        elif not self.start_time is None:

            self.elips = time.time() - self.start_time
            # self.start_time = None

            self.pub.publish(Float64(self.elips))

        else:

            self.start_time = None


if __name__ == "__main__":

    timee = time_count()

    rospy.spin()