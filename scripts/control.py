#!/usr/bin/env python

import rospy
import roslib
import tf
import math
import numpy as np


class prueba:

    def __init__(self):

        #Controller Gains
        self.k_p = 0.0
        self.k_a = 0.0
        self.k_b = 0.0

        #Speed Limits
        self.cruise_lin = 0.0
        self.cruise_lang = 0.0

        #Goal Position
        self.x_goal = 0.0
        self.y_goal = 0.0
        self.th_goal = 0.0

        #Goal transform broadcaster
        self.goal_br = tf.TransformBroadcaster()

    def set_controller_params(self):
        self.k_p = rospy.get_param('/motion_controller/gains/k_p')
        self.k_a = rospy.get_param('/motion_controller/gains/k_a')
        self.k_b = rospy.get_param('/motion_controller/gains/k_b')
        self.cruise_lin = rospy.get_param('/motion_controller/cruise/lin')
        self.cruise_ang = rospy.get_param('/motion_controller/cruise/ang')

    def broadcast_goal(self):
        self.goal_br.sendTransform((self.x_goal, self.y_goal, 0.0),
                            tf.transformations.quaternion_from_euler(0, 0, self.th_goal),
                            rospy.Time.now(),
                            "goal",
                            "base_footprint")
    
    def set_goal(self,x,y,th):
        self.x_goal = x
        self.y_goal = y
        self.th_goal = np.deg2rad(th)

        
