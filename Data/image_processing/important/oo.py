import numpy as np
import cv2
# a = np.array([[5,4],[0,1]])
# aList = [5, 2, 4, 10, 3];
# aList.sort();
# np.sort(a, axis=1)
# print "List : ", a



###################################Intitialization#############################################################
w, h = 10, 2
matrix = [[]]
Mines_locations = [[0 for x in range(h)] for y in range(w)]
Prob=[[0 for x in range(10)]for y in range(1)]
FX=[[0 for x in range(10)]for y in range(1)]
Fitness=[[0 for x in range(10)]for y in range(1)]
Sorting_Ascending_FX=[[0 for x in range(10)]for y in range(1)]
Sorting_Ascending_Coord=[[0 for x in range(h)] for y in range(w)]

M=[ [20,10],[3,4],[5,6],[1.2,7],[6.5,4]]





# print "Mines_locations: ",Mines_locations
# print "Prob: ",Prob
# print "FX: ",FX
# print "Fitness: ",Fitness
# print "Sorting_Ascending_FX: ",Sorting_Ascending_FX
# print "Sorting_Ascending_Coord: ",Sorting_Ascending_Coord
# print "Size: ",( np.shape(Mines_locations),np.shape(Prob),np.shape(FX),np.shape(Fitness),np.shape(Sorting_Ascending_FX),np.shape(Sorting_Ascending_Coord))
############################################################################################################
from math import sqrt

def dist(tuple):
    return sqrt(pow(tuple[0], 2) + pow(tuple[1], 2))

def sorting_func():
    import pdb
    pdb.set_trace()
    if dist(first) < dist(second):
        return 1
    elif dist(second) < dist(first):
        return -1
    else:
        return 0


bla = [(3, 2), (5, 4), (7, 9)]

bla.sort(sorting_func)

print bla

