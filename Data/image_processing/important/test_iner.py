from math import sqrt
import numpy as np
# import math
#
# #this loop will compute the factorial stored in variable fact
# array = [(20,10), (3,4), (5,6), (1.2,7)]
# sortList = []
# count = 0
# tempList = []
# placeholder = []
# #Compute the Equation of Minimum Value
# for x,y in array:
#     tempList.append(math.sqrt((x**2) + (y**2)))
#     tempList.append(array[count])
#     sortList.append(tempList)
#     tempList = []
#     count += 1
# print sortList
# count = 1
# placeholder  = sortList[0][:]
# print('ORIGINAL LIST\n', sortList)
# while count < (len(sortList)):
#     if sortList[count - 1][0] < sortList[count][0]:
#         print('THIS IS COUNT', count)
#         count += 1
#     else:
#         placeholder = sortList[count - 1][:]
#         print("this is placeholder: ", placeholder)
#         sortList[count - 1] = sortList[count]
#         print(sortList)
#         sortList[count] = placeholder
#         print(sortList)
#         placeholder = []
#         count = 1
# print sortList





# import math
# numbers = [[20,10], [3,4], [5,6], [1.2,7]]
# a = []
#
# for i, l in enumerate(numbers):
#     #l = the inner list
#     #i = index
#     a.append([math.sqrt(l[0]**2 + l[1]**2), i])
# print(sorted(a, key=lambda x: x[0]))
# #uncomment above to see what this does
#
# a.sort(key=lambda x: x[0])


#M = [(20, 10), (3, 4), (5, 6), (1.2, 7), (6.5, 4)]
M=[(87,22) , (70,77) , (48,76)]

def minimum_value(pair, dictionary={}):  # intentional dangerous default value
    if pair not in dictionary:

        #dictionary[pair] = pair[0]**2 + pair[1]**2
        dictionary[pair]=sqrt(pow(pair[0], 2) + pow(pair[1], 2))
    return dictionary[pair]
#M_sorted = sorted(x , key=minimum_value)

#print(M_sorted)


bla=[]
x = [ (7,8),(4,10),(5,3),(1,5),(3,2),(6,9),(7,12),(6,9) ,(3,8),(7,10),(4,7),(1,2),(2,8),(3,1),(3,14),(5,11),(6,13),(5,16) ,(7,18),(3,17) ]
x=[(6,4),(6,9),(7,3),(7,6),(8,7)]
print len(x)
X_now=[1,4,5,7,2,3,6,7]
Y_now=[2,4,5,7,6,1,2,3]

bla = sorted(x, key=minimum_value)
print bla
ss=pow(x[2][0],2)
N_O_M_L = 0
counter =0
# while(1):
#  if (N_O_M_L<len(x)):
#     values=[]
#     reso=(len(x))-N_O_M_L
#     diff_x = np.zeros (reso)
#     diff_y =  np.zeros (reso)
#     Axeo   =  np.zeros (reso)
#     Ayeo   =  np.zeros (reso)
#     ADD_all= np.zeros (reso)
#     values=  np.zeros (reso)
# # for i in range (0 ,len(x)):
# #     diff_x[i] = x[i][0]
# #     diff_y[i] = x[i][1]
# # print "diff_x :" ,  diff_x
# # print "diff_y :" ,  diff_
#
#     for i in range (0 ,reso):
#          diff_x[i] = x[i][0]   -  X_now[N_O_M_L]
#          diff_y[i] = x[i][1]   -  Y_now[N_O_M_L]
#          Axeo[i] = pow(diff_x[i], 2)
#          Ayeo[i] = pow(diff_y[i], 2)
#          ADD_all[i]= Axeo[i] + Ayeo[i]
#          values[i] = ( np.sqrt(ADD_all[i]))
#     print  "Delta_X", diff_x
#     print  "Delta_Y", diff_y
#     print  "Axeo", Axeo
#     print  "Ayeo", Ayeo
#     print  "ADD_all :", ADD_all
#     print  "values :", values
#
#
#     Minmum_value = values[0]
#     for i in range (1 ,len(values)):
#         if (values[i]<Minmum_value):
#             Minmum_value=values[i]
#             counter=counter+1
#
#     print " Minmum_value : " , Minmum_value , "__counter:",counter
#     N_O_M_L=N_O_M_L+1
#     print N_O_M_L