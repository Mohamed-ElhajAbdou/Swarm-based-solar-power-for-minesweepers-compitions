from math import sqrt
import numpy as np
import math as Mg





M=[(87,22) , (70,77) , (48,76)]


#M_sorted = sorted(x , key=minimum_value)

#print(M_sorted)
#x = [ (7,8),(18,8),(5,3),(16,5),(10,2),(14,9),(10,12),(2,9) ]
x = [ (7,8),(4,10),(5,3),(1,5),(3,2),(6,9),(7,12),(2,9) ,(3,8),(7,10),(4,7),(1,2),(2,8),(3,1),(3,14),(5,11),(6,13),(5,16) ,(7,18),(3,17) ]

X_now=[1,2,3,4,5,6,7,8,9]
Y_now=[2,3,4,5,6,7,8,9,10]
Robot =[0,5]
print Robot[1]
# v=-1
r=Mg.cos(45*(3.14/180))
print r
# Pi=Mg.pi
# #b=(v/r)
# v= 50000000
# print v
#  v
# print "fgff",v
# N_O_M_L = 0
# counter =0
# while(1):
#  if (N_O_M_L<(len(x))):
#     reso=(len(x))-N_O_M_L
#     #print "N_O_M_L",N_O_M_L ,"len(x)",len(x),"(len(x)N_O_M_L___",reso
#     values=[]
#     Minmum_value = 1000
#     counter=counter+1
#     diff_x =  np.zeros (reso)
#     diff_y =  np.zeros (reso)
#     Axeo   =  np.zeros (reso)
#     Ayeo   =  np.zeros (reso)
#     ADD_all=  np.zeros (reso)
#     values =  np.zeros (reso)
#
#     for i in range (0 ,reso):
#         diff_x[i] = 1000
#         diff_y[i] = 1000
#         Axeo[i] = 1000
#         Ayeo[i] = 1000
#         ADD_all[i] = 1000
#         values [i]= 1000
#
#     for i in range (N_O_M_L ,reso):
#          diff_x[i] = x[i][0]   -  X_now[N_O_M_L]
#          diff_y[i] = x[i][1]   -  Y_now[N_O_M_L]
#          Axeo[i] = pow(diff_x[i], 2)
#          Ayeo[i] = pow(diff_y[i], 2)
#          ADD_all[i]= Axeo[i] + Ayeo[i]
#          values[i] = ( np.sqrt(ADD_all[i]))
#          if (values[i]<Minmum_value):
#             Minmum_value=values[i]
#
#
#
#
#
#     for i in range (0 ,8):
#
#
#     for i in range (1 ,len(values)):
#         if (Minmum_value ==values[i]):
#             print i
#
#
#
#
#     print " Minmum_value : " , int ( Minmum_value )
#     print  "____X_now :",X_now[N_O_M_L] ,"___Y_now :",Y_now[N_O_M_L]
#     print "valuee:", (values)
#     print "__________________________________________________________________________________________________________"
#     print  "Delta_X", diff_x, "Delta_Y", diff_y, "Axeo", Axeo,  "Ayeo", Ayeo
#
#     N_O_M_L=N_O_M_L+1
#     print N_O_M_L
#
#
# print  "ADD_all :", ADD_all
# print  "values :", values
# # c=5
# # d=10
# # if (c>=d ):
# #     print "gff"
# # else :
# #     print "gggg"