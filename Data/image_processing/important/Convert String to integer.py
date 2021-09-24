import numpy as np
# import ast
import re
from collections import Counter
def pop_method():
    global a
    global index
    a.pop(index)
########________charcter generator__________################################
def character_range(a, b, inclusive=False):
    back = chr
    if isinstance(a,unicode) or isinstance(b,unicode):
        back = unicode
    for c in xrange(ord(a), ord(b) + int(bool(inclusive))):
        yield back(c)



#for c in character_range('A', 'H', inclusive=True):
    #print((c))
#x = [ (5,3),(4,5),(4,2),(3,4) ]
#print x[1][0]
x = [ (7,8),(4,10),(5,3),(1,5),(3,2),(6,9),(7,12),(2,9) ,(3,8),(7,10),(4,7),(1,2),(2,8),(3,1),(3,14),(5,11),(6,13),(5,16) ,(7,18),(3,17) ]
x3=[[0 for col in range (len(x))] for row in range (2)]

#x3= sorted(x, key=minimum_value)
print x3
x2=zip(*x)

i=0
j=0
dode1=[[0 for col in range (len(x))] for row in range (2)]

# print dode
# for i in range (0,len(x)):
#     for j in range(1):
#      if (x[i][0]<5):
#        print dode1[i][j] = x[i][j]









for i in range (0,len(x)):
    for j in range (2):
      if (x[i][0]>5):
        if x2[j][i]==0:
            dode1[j][i]='A'

        if x2[j][i] == 1:
            dode1[j][i] = 'B'

        if x2[j][i] == 2:
            dode1[j][i] = 'C'


        if x2[j][i] == 3:
            dode1[j][i] = 'D'

        if x2[j][i] == 4:
            dode1[j][i] = 'E'

        if x2[j][i] == 5:
            dode1[j][i] = 'F'

        if x2[j][i] == 6:
            dode1[j][i] = 'G'

        if x2[j][i] == 7:
            dode1[j][i] = 'H'

        if x2[j][i] == 8:
            dode1[j][i] = 'I'

        if x2[j][i] == 9:
            dode1[j][i] = 'J'

        if x2[j][i] == 10:
            dode1[j][i] = 'K'


        if x2[j][i] == 11:
            dode1[j][i] = 'L'

        if x2[j][i] == 12:
            dode1[j][i] = 'M'

        if x2[j][i] == 13:
            dode1[j][i] = 'N'

        if x2[j][i] == 14:
            dode1[j][i] = 'O'

        if x2[j][i] == 15:
            dode1[j][i] = 'P'

        if x2[j][i] == 16:
            dode1[j][i] = 'Q'

        if x2[j][i] == 17:
            dode1[j][i] = 'R'

        if x2[j][i] == 18:
            dode1[j][i] = 'S'

      if (x[i][0] < 5):
        if x2[j][i] == 0:
            x3[j][i] = 'A'

        if x2[j][i] == 1:
            x3[j][i] = 'B'

        if x2[j][i] == 2:
            x3[j][i] = 'C'

        if x2[j][i] == 3:
            x3[j][i] = 'D'

        if x2[j][i] == 4:
            x3[j][i] = 'E'

        if x2[j][i] == 5:
            x3[j][i] = 'F'

        if x2[j][i] == 6:
            x3[j][i] = 'G'

        if x2[j][i] == 7:
            x3[j][i] = 'H'

        if x2[j][i] == 8:
            x3[j][i] = 'I'

        if x2[j][i] == 9:
            x3[j][i] = 'J'

        if x2[j][i] == 10:
            x3[j][i] = 'K'

        if x2[j][i] == 11:
            x3[j][i] = 'L'

        if x2[j][i] == 12:
            x3[j][i] = 'M'

        if x2[j][i] == 13:
            x3[j][i] = 'N'

        if x2[j][i] == 14:
            x3[j][i] = 'O'

        if x2[j][i] == 15:
            x3[j][i] = 'P'

        if x2[j][i] == 16:
            x3[j][i] = 'Q'

        if x2[j][i] == 17:
            x3[j][i] = 'R'

        if x2[j][i] == 18:
            x3[j][i] = 'S'


print "Less_than_5",x3
print "More_than_5",dode1
# koke=[]
# W=0
message1=[]
message2=[]
message3=[]

x3=zip(*x3)
dode1=zip(*dode1)
message = list(x3)
message2=list(dode1)
print"List_Less_than_5", message
print"List_More_than_5", message2



Final_Array_1=[[0 for col in range (len(x))] for row in range (2)]
Final_Array_2=[[0 for col in range (len(x))] for row in range (2)]

Q1=0
Q2=0

print len(message)

for i in range (0,len(message)):

        if message[i][0] !=0 and message[i][1] !=0:
            Final_Array_1[0][Q1]=message[i][0]
            Final_Array_1[1][Q1] = message[i][1]
            Q1 = Q1 + 1

        if message2[i][0] != 0 and message2[i][1] != 0:
            Final_Array_2[0][Q2]=message2[i][0]
            Final_Array_2[1][Q2] = message2[i][1]
            Q2 = Q2 + 1

Final_Array_1=zip(*Final_Array_1)
Final_Array_2=zip(*Final_Array_2)
#Final_Array_1.append("X")
Final_Array_1.insert(0,"a")
Final_Array_2.insert(0,"b")


print Final_Array_1
print Final_Array_2

flag =0
flag=  (flag ^1 )
print flag
flag=  (flag ^1 )
print flag
flag=  (flag ^1 )
print flag



# for i in range (0,len(message1)):
#
#         if message1[i]=='A':
#             message2[i]=0
#
#         if message1[i]=='B':
#             message2[i]=1
#
#         if message1[i]=='C':
#             message2[i]=2
#
#         if message1[i]=='D':
#             message2[i]=3
#
#         if message1[i]=='E':
#             message2[i]=4
#
#         if message1[i]=='F':
#             message2[i]=5
#
#         if message1[i]=='G':
#             message2[i]=6
#
#         if message1[i]=='H':
#             message2[i]=7
#
#         if message1[i]=='I':
#             message2[i]=8
#
#         if message1[i]=='J':
#             message2[i]=9
#
#         if message1[i]=='K':
#             message2[i]=10
#
#         if message1[i]=='L':
#             message2[i]=11
#
#         if message1[i]=='M':
#             message2[i]=12
#
#         if message1[i]=='N':
#             message2[i]=13
#
#         if message1[i]=='O':
#             message2[i]=14
#
#         if message1[i]=='P':
#             message2[i]=15
#
#         if message1[i]=='Q':
#             message2[i]=16
#
#         if message1[i]=='R':
#             message2[i]=17
#
#         if message1[i]=='S':
#             message2[i]=18

# print message1,len(message1)
# print message2 ,len(message2)
#dode=np.zeros(len(message2))
# for i in range ( 0,(len(message2))-1):
#     if (i%2==0):
#         if message2[i]<5:
#          dode[i]==message2[i]
#
#          print message2[i] ,i
#          print x
#          print "Filter ",x[i]
#








#print x3[1][0]

# def char_range(c1, c2):
#     """Generates the characters from `c1` to `c2`, inclusive."""
#     for c in xrange(ord(c1), ord(c2)+1):
#         yield chr(c)
#




#tub='n'.join(map(str,x3))
#print message , message[1]
#print message.pop(1)

# map(str.split,x3)
#
# for i in range (0, len(x3)):
#     if x3[i]==")" :
#         print "tes"
#         x3.remove(x3[i])
#

# bad_words = ['and']
#
#
# essay_text = 'This, this, TAAAAAAis aDDDDd that.'
#     # This regular expression finds consecutive strings of lowercase letters.
#     # Counter counts each unique string and collects them in a dictionary.
# result = Counter(re.findall(r'[A-D]+',essay_text.upper()))
# print result




