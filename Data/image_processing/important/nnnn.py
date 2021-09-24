import math
array = [(20,10), (3,4), (5,6), (1.2,7)]
sortList = []
count = 0
tempList = []
placeholder = []
#Compute the Equation of Minimum Value
for x,y in array:
    tempList.append(math.sqrt((x**2) + (y**2)))
    tempList.append(array[count])
    sortList.append(tempList)
    tempList = []
    count += 1
sortList
count = 1
placeholder  = sortList[0][:]
print('ORIGINAL LIST\n', sortList)
while count < (len(sortList)):
    if sortList[count - 1][0] < sortList[count][0]:
        print('THIS IS COUNT', count)
        count += 1
    else:
        placeholder = sortList[count - 1][:]
        print("this is placeholder: ", placeholder)
        sortList[count - 1] = sortList[count]
        print(sortList)
        sortList[count] = placeholder
        print(sortList)
        placeholder = []
        count = 1

print(sortList)
