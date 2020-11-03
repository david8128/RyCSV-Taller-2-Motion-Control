#!/usr/bin/env python

import rospy
import roslib
import tf_conversions
import tf2_ros
import math
import numpy as np
import time
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist


class Motion:
    
    def __init__(self):

        #Controller Gains
        self.k_p = 0.0
        self.k_a = 0.0
        self.k_b = 0.0  

        #Speed Limits
        self.cruise_lin = 0.0
        self.cruise_ang = 0.0

        #Goal Position
        self.x_goal = 0.0
        self.y_goal = 0.0
        self.th_goal = 0.0

        #Goal transform broadcaster
        self.goal_br = tf2_ros.TransformBroadcaster()

        #Errors
        self.error_x = 0.0
        self.error_y = 0.0
        self.error_th = 0.0

        #Error transform broadcaster
        self.tfBuffer = tf2_ros.Buffer()
        self.error_listener = tf2_ros.TransformListener(self.tfBuffer)

        #Error publisher (Debuging)
        self.error_pub = rospy.Publisher('error', TransformStamped, queue_size=10)

        #Polar coordinates error
        self.alpha = 0.0
        self.beta = 0.0
        self.p = 0.0

        #Controller Vector
        self.k_vector = np.array([[-1*self.k_p*self.p*math.cos(self.alpha),
                                   (self.k_p*math.sin(self.alpha))-(self.k_a*self.alpha)-(self.beta*self.k_b),
                                    -1*self.k_p*math.sin(self.alpha)]])

        #Polar speeds
        self.alpha_dot = 0.0
        self.beta_dot = 0.0
        self.p_dot = 0.0

        #Speeds (Controller outs)
        self.v_out = 0.0
        self.w_out = 0.0

    def set_controller_params(self,config, level):
        #Get control parameters from ROS param server
        #This function is a callback from the dynamic reconfigure server
        self.k_p = rospy.get_param('/motion_controller/k_p')
        self.k_a = rospy.get_param('/motion_controller/k_a')
        self.k_b = rospy.get_param('/motion_controller/k_b')
        self.cruise_lin = rospy.get_param('/motion_controller/cruise/lin')
        self.cruise_ang = rospy.get_param('/motion_controller/cruise/ang')
        print('p gain: '+str(self.k_p))
        print('a gain: '+str(self.k_a))
        print('b gain: '+str(self.k_b))
        return(config)
        

    def broadcast_goal(self,now):
        #Broadcast goal position as TF transform

        t = TransformStamped()

        t.header.stamp = now
        t.header.frame_id = "odom" #Fixed Frame
        t.child_frame_id = "goal" #Goal frame
        t.transform.translation.x = self.x_goal 
        t.transform.translation.y = self.y_goal
        t.transform.translation.z = 0.0
        #Transform from rpy (angles) to quaternion
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.th_goal)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.goal_br.sendTransform(t) #Broadcast
    
    def set_goal(self,x,y,th):
        self.x_goal = x
        self.y_goal = y
        self.th_goal = np.deg2rad(th)

    def compute_error(self,now):
        #Measure error from base footprint to goal
        try:
            trans = self.tfBuffer.lookup_transform('goal', 'base_footprint', now, rospy.Duration(1.0))
            quat = np.zeros(4)
            quat[0] = trans.transform.rotation.x 
            quat[1] = trans.transform.rotation.y 
            quat[2] = trans.transform.rotation.z 
            quat[3] = trans.transform.rotation.w 
            #Transform from quaternion to rpy
            rpy = tf_conversions.transformations.euler_from_quaternion(quat)
            self.error_x = trans.transform.translation.x * -1 #Negative (measured from goal to base, inverted)
            self.error_y = trans.transform.translation.y * -1
            self.error_th = rpy[2]  #Orientation respecting right hand rule
            self.error_pub.publish(trans) #Publish error in topic for debugging
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Exception")
    
    def transform_error(self):
        #Transform error from X-Y cartesian cordinates to polar coordinates
        #Refer to: Roland Siegwart, Intro to autonomus mobile robots - Chap 3 (Control Law)
        self.p = math.sqrt((pow(self.error_x,2) + pow(self.error_y,2)))
        self.alpha = (-1 * self.error_th) + math.atan2(self.error_y,self.error_x)
        self.beta = (-1 * self.error_th) - (self.alpha)
        print(" - - - - - - ")
        print(" CONVERSION A POLARES")
        print("Polares alpha: "+str(self.alpha))
        print("Polares beta: "+str(self.beta))
        print("Polares p: "+str(self.p))
        print(" - - - - - - ")

    def control_speed(self):

        #Control Law
        #Refer to: Roland Siegwart, Intro to autonomus mobile robots - Chap 3 (Control Law)
        v = self.p * self.k_p                               #Linear speed
        w = self.k_a * self.alpha + self.k_b * self.beta    #Angular speed
        
        #Angular speed limit
        self.w_out = np.sign(w)*min(abs(w),np.deg2rad(self.cruise_ang))

        #Lineal speed limit and reverse correction
        if(self.alpha <= (np.pi/2) or self.alpha >= (-np.pi/2)):
            self.v_out = min(v,self.cruise_lin)     
        else:
            self.v_out = max(-1*v,-1*self.cruise_lin) 

        print(" - - - - - - ")
        print(" SALIDA CONTROLADOR (TRANSFORMADA)")
        print("V out :"+str(self.v_out))
        print("W out :"+str(self.w_out))
        print(" - - - - - - ")

    def arrived2goal(self):
        #Check is robot base has arrived to goal
        if (abs(self.error_x)<0.02 and abs(self.error_y)<0.02 and abs(self.error_th)<0.02):
            return True
        else:
            return False