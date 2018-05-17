#!/bin/python

import numpy as np
import pylab as pl

def readData(aux):
    fs = open(aux)
    ls = [item.split() for item in fs]
    v = np.array(ls)
    v = np.double(v)
    return v
    

file_dir = '/home/solomon/STONE/ROS/pioneering_ws/src/sprite_robo/trajectory_planning/data/'
aux = file_dir + 'x_cubic_spline_waypoints.txt'
#aux = file_dir + 'cubic_start2goal.txt'
#aux = file_dir + 'triangular_new.txt'
#aux = file_dir + 'trapezoidal_new.txt'

v = readData(aux)

aux = file_dir + 'y_cubic_spline_waypoints.txt'
v2 = readData(aux)

pl.close('all')

f, axearr = pl.subplots(3,1,sharex=True)

axearr[0].plot(v[:,0], label='positionX')
axearr[0].hold(True)
axearr[0].plot(v2[:,0], label='positionY')
axearr[0].legend(loc = "upper right")
axearr[0].grid(True)

axearr[1].plot(v[:,1], label='velocityX')
axearr[1].hold(True)
axearr[1].plot(v2[:,1], label='velocityY')
axearr[1].legend(loc = "upper right")
axearr[1].grid(True)

axearr[2].plot(v[:,2],label='accelerationX')
axearr[2].hold(True)
axearr[2].plot(v[:,2],label='accelerationY')
axearr[2].legend(loc='upper right')
axearr[2].grid(True)


#pl.figure()
#pl.subplot(2,1,1)
#pl.plot(v[:,0],'r*', label='speed')
##pl.hold(True)
#pl.subplot(2,2,2)
#pl.figure()
#pl.plot(v[:,1], 'b*', label='disp')

pl.show()
