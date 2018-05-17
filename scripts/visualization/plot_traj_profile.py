#!/bin/python

import numpy as np
import pylab as pl

file_dir = './'
aux = file_dir + 'cubic_spline_waypoints.txt'
#aux = file_dir + 'cubic_start2goal.txt'
#aux = file_dir + 'triangular_new.txt'
#aux = file_dir + 'trapezoidal_new.txt'

fs = open(aux)
ls = [item.split() for item in fs]
v = np.array(ls)
v = np.double(v)

pl.close('all')

f, axearr = pl.subplots(3,1,sharex=True)

axearr[0].plot(v[:,0], label='position')
axearr[0].legend(loc = "upper right")
axearr[0].grid(True)

axearr[1].plot(v[:,1], label='velocity')
axearr[1].legend(loc = "upper right")
axearr[1].grid(True)

axearr[2].plot(v[:,2],label='acceleration')
axearr[2].legend(loc='upper right')
axearr[2].grid(True)

pl.show()
