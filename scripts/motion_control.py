#!/usr/bin/env python

#Lib imports
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
        self.cruise_lang = 0.0

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

    def set_controller_params(self):
        self.k_p = rospy.get_param('/motion_controller/gains/k_p')
        self.k_a = rospy.get_param('/motion_controller/gains/k_a')
        self.k_b = rospy.get_param('/motion_controller/gains/k_b')
        self.cruise_lin = rospy.get_param('/motion_controller/cruise/lin')
        self.cruise_ang = rospy.get_param('/motion_controller/cruise/ang')
        
        print("Matriz de control")
        print(self.k_vector)

    def broadcast_goal(self):
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "goal"
        t.transform.translation.x = self.x_goal
        t.transform.translation.y = self.y_goal
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.th_goal)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.goal_br.sendTransform(t)
    
    def set_goal(self,x,y,th):
        self.x_goal = x
        self.y_goal = y
        self.th_goal = np.deg2rad(th)

    def compute_error(self):
        try:
            now = rospy.Time(0)
            trans = self.tfBuffer.lookup_transform('goal', 'base_footprint', now, rospy.Duration(1.0))
            quat = np.zeros(4)
            quat[0] = trans.transform.rotation.x 
            quat[1] = trans.transform.rotation.y 
            quat[2] = trans.transform.rotation.z 
            quat[3] = trans.transform.rotation.w 
            rpy = tf_conversions.transformations.euler_from_quaternion(quat)
            self.error_x = trans.transform.translation.x * -1
            self.error_y = trans.transform.translation.y * -1
            self.error_th = rpy[2] 
            self.error_pub.publish(trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Exception")
    
    def transform_error(self):
        self.p = math.sqrt((pow(self.error_x,2) + pow(self.error_y,2)))
        self.alpha = (-1 * self.error_th) + math.atan2(self.error_y,self.error_x)
        self.beta = (-1 * self.error_th) - (self.alpha)
        print(" - - - - - - ")
        print(" CONVERSION A POLARES")
        print("Polares alpha: "+str(self.alpha))
        print("Polares beta: "+str(self.beta))
        print("Polares p: "+str(self.p))
        print(" - - - - - - ")

    def compute_control(self):
        self.k_vector = np.array([[-1*self.k_p*self.p*math.cos(self.alpha),
                                   (self.k_p*math.sin(self.alpha))-(self.k_a*self.alpha)-(self.beta*self.k_b),
                                    -1*self.k_p*math.sin(self.alpha)]])
        self.p_dot = self.k_vector[0,0]
        self.alpha_dot = self.k_vector[0,1]
        self.beta_dot = self.k_vector[0,2]
        print(" - - - - - - ")
        print(" SALIDA CONTROLADOR (EN POLAR)")
        print("Vel alpha: "+str(self.alpha_dot))
        print("Vel beta: "+str(self.beta_dot))
        print("Vel p: "+str(self.p_dot))
        print(" - - - - - - ")

    def transform_speed(self):
        ts = np.array([[math.cos(self.alpha),0],
                      [-1*(math.sin(self.alpha)/self.p),1],
                      [math.sin(self.alpha)/self.p,0]])
        polar = np.array([[self.p_dot],[self.alpha_dot],[self.beta_dot]])
        out = np.matmul(np.linalg.pinv(ts),polar)
        print(" - - - - - - ")
        print(" SALIDA CONTROLADOR (TRANSFORMADA)")
        print(out)
        print(" - - - - - - ")
        self.v_out = -1*out[0,0]
        self.w_out = out[1,0]

if __name__ == '__main__':

    rospy.init_node('motion_controller', anonymous=True)
    rospy.loginfo("Motion controller node init")

    nameSpeedTopic = "/mobile_base/commands/velocity"
    kobuki_speed_pub = rospy.Publisher(nameSpeedTopic, Twist, queue_size=10)

    controlador = Motion()
    controlador.set_controller_params()
    controlador.set_goal(5,5,70)

    rate = rospy.Rate(20) # 20 Hz

    print("WAITING FOR GAZEBO")
    time.sleep(4)

    command = Twist()

    while (not rospy.is_shutdown()):
        controlador.broadcast_goal()
        controlador.compute_error()
        controlador.transform_error()
        controlador.compute_control()
        controlador.transform_speed()

        command.linear.x =  controlador.v_out
        command.angular.z =  controlador.w_out

        kobuki_speed_pub.publish(command)

        rate.sleep()