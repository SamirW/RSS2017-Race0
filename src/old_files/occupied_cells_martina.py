# -*- coding: utf-8 -*-
"""
Created on Fri Apr  7 16:35:59 2017

@author: clemmie
"""

import numpy as np
from scipy.misc import imread


map_img = imread('../maps/basement_fixed_more_dilated.png')

#rows = len(map_img)
#cols = len(map_img[0])

#map_occupancy = np.empty((rows,cols),dtype='int')
available_cells = set()

for i in range(len(map_img)):
    for j in range(len(map_img[0])):
        if map_img[i][j] == 255:
            available_cells.add((i,j))
            
#np.savetxt('occupied_cells.txt', occupied_cells)

#occupied_cells_array = np.loadtxt('avail_cells_np.txt')
#print occupied_cells
print "completed"

print available_cells