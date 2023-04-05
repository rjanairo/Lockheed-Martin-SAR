import math

GRID_SIZE = 1 #size of grid in meters

MAP_SIZE = 20 #How many Grid tall and wide the map is

NUM_DOTS = 10 #number of points along the line to search

rotationEvenOdd = 0

map = [[0 for x in range(MAP_SIZE)] for y in range(MAP_SIZE)] #initilize with 0s
#Map key:
#0 = unknown
#1 = clear
#2 = obstical
#3 = potential obstical

#Map [0][0] is top left corner
#Up is map[x--][y]
#right is [x][y++]

#startRow and startCol refers to the origin of the ray
#end refers to the end of the ray
#result: 2 if the ray hits, 1 if no hit
#evenOdd: a bool used to dertime if the rotation is on the same rotation
def updateMap(startRow, startCol, endRow, endCol, result, evenOdd):
    rotationEvenOdd = evenOdd

    #stupid code that just draws X amount of points along the line and checks that
    ROW_INCREMENT = (endRow-startRow)/NUM_DOTS
    COL_INCREMENT = (endCol-startCol)/NUM_DOTS

    PointRow = startRow; #this is the location of the point that we are checking
    PointCol = startCol;

    for i in range(0, NUM_DOTS-1):
        if(map[math.floor(PointRow)][math.floor(PointCol)] == 0): #mark as clear if square is unexplored
            map[math.floor(PointRow)][math.floor(PointCol)] = 1
        elif(map[math.floor(PointRow)][math.floor(PointCol)] == 2 and not (rotationEvenOdd == evenOdd)): #mark clear if square is obstical only if its a seperate rotation
            map[math.floor(PointRow)][math.floor(PointCol)] = 2

        PointRow += ROW_INCREMENT
        PointCol += COL_INCREMENT

    if(map[math.floor(PointRow)][math.floor(PointCol)] == 0): #mark as clear if square is unexplored
        map[math.floor(PointRow)][math.floor(PointCol)] = result
    elif(map[math.floor(PointRow)][math.floor(PointCol)] == 2 and not (rotationEvenOdd == evenOdd)): #mark clear if square is obstical only if its a seperate rotation
        map[math.floor(PointRow)][math.floor(PointCol)] = result

    rotationEvenOdd = evenOdd;
