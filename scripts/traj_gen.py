#!/usr/bin/env python

import math
import numpy as np
import time

class interpolator:

    def __init__(self,p1, p2):
        self.p1 = p1
        self.p2 = p2

    def compute_traj(self,n):
        x_init = self.p1[0]
        y_init = self.p1[1]
        th_init = self.p1[2]

        d_x = self.p2[0] - self.p1[0]
        d_y = self.p2[1] - self.p1[1]
        th = self.p2[2] - self.p1[2]
        r = np.abs(d_y/math.sin(th))
        
        d_th = th/n

        traj = np.zeros((n+1,3))

        print("x_init: "+ str(x_init))
        print("r: "+ str(r))

        for x in range(traj.shape[0]):
            print("Iterator: " + str(x))
            if(d_x > 0):
                x_temp = x_init + (d_x - np.sign(d_th)*r*(math.cos(x*d_th)))
                print("pos")
            else: 
                x_temp = x_init - (abs(d_x) - r*(math.cos(x*d_th)))
                print("neg")
            print("X: " + str(x_temp))
            y_temp = y_init + r*(math.sin(x*d_th))
            print("Y: " + str(y_temp))
            th_temp = th_init + x*d_th
            print("TH: " + str(th_temp))
            traj[x] = [x_temp,y_temp,th_temp]

        return traj                

if __name__ == '__main__':

    p1 = [2,2,np.deg2rad(90)]
    p2 = [3,3,np.deg2rad(0)]

    inter = interpolator(p1,p2)

    dots = inter.compute_traj(5)
    print(dots)