#!/usr/bin/env python

#Lib imports
import rospy
import roslib
import tf_conversions
import tf2_ros
import math
import numpy as np
import time
from control import Motion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from rycsv_kobuki_motion_control.cfg import controllerConfig

def angle_between(p0,p1,p2):
    v0 = np.array(p1) - np.array(p0)
    v1 = np.array(p2) - np.array(p0)

    angle = np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1))
    return angle

def xy2traj(dots):
    """
    From xy coordinates generates a complete trajectory

    Takes x,y coordinates of a trajectory and calculates x,y,theta coordinates
    with intermidiate points that assure twists of 90
    
    Parameters
    ----------
    dots : list of [x,y]
        Dots [[x_0,y_0],[x_1,y_1],...]
    Motion : control.Motion
        Class where the control and motion is settled

    Returns
    -------
    list of [x,y,theta]
        Complete trajectory

    """
    traj = []
    last_dot = 0 
    last_x = 0
    last_y = 0
    for count, dot in enumerate(dots):
        x = dot[0]
        y = dot[1]
        if (count == 0) :
            theta = 90
            traj.append([x,y,theta])           #Radians to deg
        else:
            theta = angle_between(last_dot,[last_x+1,last_y],dot)
            traj.append([last_x,last_y,np.rad2deg(theta)])
            traj.append([x,y,np.rad2deg(theta)])
        last_dot = dot
        last_theta = theta
        last_x = x
        last_y = y
    return traj

    
if __name__ == '__main__':

    #Trajectory node init
    rospy.init_node('motion_controller', anonymous=True)
    rospy.loginfo("Motion controller node init")

    #Wheel speed publisher
    nameSpeedTopic = "/mobile_base/commands/velocity"
    kobuki_speed_pub = rospy.Publisher(nameSpeedTopic, Twist, queue_size=10)
    command = Twist()

    #Controller object
    controlador = Motion()

    #Trajectory dots (No orientation)
    dots = [
            [0, 0], [-3.5, 0], [-3.5, 3.5], [1.5, 3.5],
            [1.5, -1.5], [3.5, -1.5], [3.5, -8.0], 
            [-2.5, -8.0], [-2.5, -5.5], [1.5, -5.5], 
            [1.5, -3.5], [-1.0, -3.5]
           ]

    #Add orientation to trajectory
    traj = xy2traj(dots)
    traj = np.array(traj)
    #Trajectory limits
    goal_id = 0
    dot_count, coord = traj.shape

    rate = rospy.Rate(20) # 20 Hz ROS

    print("WAITING FOR GAZEBO")
    rospy.wait_for_service('/gazebo/spawn_urdf_model') #Wait for model spawn
    time.sleep(2)
    
    #Initial point
    controlador.set_goal(traj[goal_id][0],traj[goal_id][1],traj[goal_id][2])

    #Square trajectory for testing
    #controlador.set_goal(0,0,0)
    #flag = 0
    #square = np.array([[0,0,0],
    #                   [2,0,0],
    #                   [2,0,90],
    #                   [2,2,90],
    #                   [2,2,180],
    #                   [0,2,180],
    #                   [0,2,270],
    #                   [0,0,270]])

    #Server to get param from dynamic reconfig
    srv = Server(controllerConfig, controlador.set_controller_params)
    

    while (not rospy.is_shutdown()):

        #Get "now" time to syncronize target tf and error tf 
        now = rospy.Time.now()
        controlador.broadcast_goal(now) 
        controlador.compute_error(now)
        controlador.transform_error()
        controlador.control_speed() #Compute wheel speed (out)

        command.linear.x =  controlador.v_out
        command.angular.z =  controlador.w_out

        kobuki_speed_pub.publish(command)

        if controlador.arrived2goal():  
            goal_id = goal_id+1      #Change point when arrived to goal

            if goal_id == dot_count:
                goal_id = 0         #Go back to initial point

            #Square testing
            #controlador.set_goal(square[flag,0],square[flag,1],square[flag,2])
            controlador.set_goal(traj[goal_id][0],traj[goal_id][1],traj[goal_id][2])

        rate.sleep()